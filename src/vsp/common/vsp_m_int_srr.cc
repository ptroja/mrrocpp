// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (VSP)
// Plik:            vsp_m_nint.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Interaktywna powloka procesow VSP 
// 
// 	-	interaktywne odczytywanie stanu czujnika rzeczywistego, oczekiwanie na zakonczenie operacji
// 	-	jednowatkowy
//
// Autor:		ptrojane
// Data:		11.10.2007
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_sensor.h"				// zawiera deklaracje klasy vsp_sensor + struktury komunikacyjne
#include "messip/messip.h"

// Konfigurator
#include "lib/configurator.h"

/********************************* GLOBALS **********************************/
vsp_sensor *vs;		// czujnik wirtualny

sr_vsp *sr_msg;		// komunikacja z SR

configurator* config;

bool TERMINATE = false;									// zakonczenie obu watkow

/********************************** SIGCATCH ********************************/
void catch_signal(int sig) {
  switch(sig) {
	case SIGTERM :
	  TERMINATE = true;
	  vs->terminate();
	  _exit(EXIT_SUCCESS);
	  break;
	case SIGSEGV:
	  fprintf(stderr, "Segmentation fault in VSP process\n");
	  signal(SIGSEGV, SIG_DFL);
	  break;
  } // end: switch
}

/***************************** ERROR_HANDLER ******************************/
template<class ERROR>
void error_handler(ERROR e){
	switch(e.error_class){
		case SYSTEM_ERROR:
			printf("VSP aborted due to SYSTEM_ERROR\n");
			sr_msg->message (SYSTEM_ERROR, e.error_no);
			TERMINATE=true;
			break;
		case FATAL_ERROR:
			sr_msg->message (FATAL_ERROR, e.error_no);
			break;
		case NON_FATAL_ERROR:
			switch(e.error_no){
			case INVALID_COMMAND_TO_VSP:
				vs->from_vsp.vsp_report=INVALID_VSP_COMMAND;
				sr_msg->message (NON_FATAL_ERROR, e.error_no);
			break;
			case SENSOR_NOT_CONFIGURED:
				vs->from_vsp.vsp_report=VSP_SENSOR_NOT_CONFIGURED;
				sr_msg->message (NON_FATAL_ERROR, e.error_no);
				break;
			case READING_NOT_READY:
				vs->from_vsp.vsp_report=VSP_READING_NOT_READY;
				break;
			default:
				sr_msg->message (NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
			}; // end switch
			break;
		default:
			sr_msg->message (NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
		} // end switch  
	} // end error_handler

/*********************************** MAIN ***********************************/
int main(int argc, char *argv[]) {
    char* attach_point;
	messip_channel_t *ch;
    	
	// ustawienie priorytetow
	//setprio(getpid(), MAX_PRIORITY-3); 
	// wylapywanie sygnalow
	signal(SIGTERM, &catch_signal);
	signal(SIGSEGV, &catch_signal);
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif

	// liczba argumentow
#if defined(PROCESS_SPAWN_YRSH)
	if(argc <=6){
		printf("Za malo argumentow VSP\n");
		return -1;
		};
#endif
	 // zczytanie konfiguracji calego systemu
 	config = new configurator(argv[1], argv[2], argv[3], argv[4], argv[5]);
	if (argc>6) {
 		config->answer_to_y_rsh_spawn(argv[6]); 
 		signal(SIGINT, SIG_IGN);
 	}
	attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "resourceman_attach_point");

	try {
		/* Lokalizacja procesu wywietlania komunikatow SR */ 
		if ((sr_msg = new sr_vsp(VSP, attach_point,
			 config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]"))) == NULL) 
		{
			printf("communication with SR not ready\n");
		}
		else
			sr_msg->message ("Communication with SR ready");

		// Stworzenie nowego czujnika za pomoca funkcji (cos na ksztalt szablonu abstract factory).
		vs = return_created_sensor();

		if ((ch = messip_channel_create(NULL, attach_point, MESSIP_NOTIMEOUT, 0)) == NULL) {
			perror("messip_channel_create()");
		}

		/* start the resource manager message loop */
		sr_msg->message ("Device is waiting for clients...");
		while(!TERMINATE) { // for(;;)

			int32_t type, subtype;
			int rcvid;
	
			rcvid = messip_receive(ch, &type, &subtype, &(vs->to_vsp), sizeof(vs->to_vsp), MESSIP_NOTIMEOUT);
	
			if (rcvid == -1) /* Error condition, exit */
			{
				perror("VSP: Receive failed\n");
				break;
			} else if (rcvid < -1) {
				// ie. MESSIP_MSG_DISCONNECT
				fprintf(stderr, "VSP: ie. MESSIP_MSG_DISCONNECT\n");
				continue;
			}
	
			vs->from_vsp.vsp_report=VSP_REPLY_OK;
	
			try {
				switch(vs->to_vsp.i_code) {
					case VSP_CONFIGURE_SENSOR :
						vs->configure_sensor();
						break;
					case VSP_INITIATE_READING :
						vs->initiate_reading();
						break;
					case VSP_GET_READING :
						vs->get_reading();
						break;
					case VSP_TERMINATE :
						vs->terminate();
						TERMINATE=true;
						break;
					default :
						throw VSP_main_error(NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
				}
			}

			catch (VSP_main_error e){
				error_handler(e);
			} // end CATCH
			catch (sensor::sensor_error e){
				error_handler(e);
			} // end CATCH
		
			messip_reply(ch, rcvid, 0, &vs->from_vsp, sizeof(VSP_REPORT) + vs->union_size, MESSIP_NOTIMEOUT);
 		} // end while()
        sr_msg->message ("VSP terminated");
	} // koniec TRY
	catch (VSP_main_error e) {
		error_handler(e);
		exit(EXIT_FAILURE);
	} // end CATCH
} // end MAIN
