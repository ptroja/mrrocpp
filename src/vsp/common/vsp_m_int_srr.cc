// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
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

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_sensor.h"				// zawiera deklaracje klasy vsp_sensor + struktury komunikacyjne
#include "messip/messip.h"

// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace vsp {
namespace common {

/********************************* GLOBALS **********************************/
sensor::base *vs;		// czujnik wirtualny

// lib::sr_vsp *vs->sr_msg;		// komunikacja z SR

// configurator* config;

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
		case lib::SYSTEM_ERROR:
			printf("VSP aborted due to lib::SYSTEM_ERROR\n");
			vs->sr_msg->message (lib::SYSTEM_ERROR, e.error_no);
			TERMINATE=true;
			break;
		case lib::FATAL_ERROR:
			vs->sr_msg->message (lib::FATAL_ERROR, e.error_no);
			break;
		case lib::NON_FATAL_ERROR:
			switch(e.error_no){
			case INVALID_COMMAND_TO_VSP:
				vs->from_vsp.vsp_report= lib::INVALID_VSP_COMMAND;
				vs->sr_msg->message (lib::NON_FATAL_ERROR, e.error_no);
			break;
			case SENSOR_NOT_CONFIGURED:
				vs->from_vsp.vsp_report= lib::VSP_SENSOR_NOT_CONFIGURED;
				vs->sr_msg->message (lib::NON_FATAL_ERROR, e.error_no);
				break;
			case READING_NOT_READY:
				vs->from_vsp.vsp_report= lib::VSP_READING_NOT_READY;
				break;
			default:
				vs->sr_msg->message (lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
			}; // end switch
			break;
		default:
			vs->sr_msg->message (lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
		} // end switch  
	} // end error_handler

} // namespace common
} // namespace vsp
} // namespace mrrocpp

/*********************************** MAIN ***********************************/
int main(int argc, char *argv[]) {
    char* attach_point;
	messip_channel_t *ch;
    	
	// ustawienie priorytetow
	//setprio(getpid(), MAX_PRIORITY-3); 
	// wylapywanie sygnalow
	signal(SIGTERM, &vsp::common::catch_signal);
	signal(SIGSEGV, &vsp::common::catch_signal);
#if defined(PROCESS_SPAWN_RSH)
	signal(SIGINT, SIG_IGN);
#endif

	// liczba argumentow

	if(argc <=6){
		printf("Za malo argumentow VSP\n");
		return -1;
		};

	 // zczytanie konfiguracji calego systemu
	lib::configurator *_config = new lib::configurator(argv[1], argv[2], argv[3], argv[4], argv[5]);
#if defined(PROCESS_SPAWN_YRSH)
	if (argc>6) {
 		config->answer_to_y_rsh_spawn(argv[6]); 
 		signal(SIGINT, SIG_IGN);
 	}
#endif
	attach_point = _config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point");

	try {
	
		// Stworzenie nowego czujnika za pomoca funkcji (cos na ksztalt szablonu abstract factory).
		vsp::common::vs = vsp::sensor::return_created_sensor(*_config);

		if ((ch = messip_channel_create(NULL, attach_point, MESSIP_NOTIMEOUT, 0)) == NULL) {
			perror("messip_channel_create()");
		}

		/* start the resource manager message loop */
		vsp::common::vs->sr_msg->message ("Device is waiting for clients...");
		while(!vsp::common::TERMINATE) { // for(;;)

			int32_t type, subtype;
			int rcvid;
	
			rcvid = messip_receive(ch, &type, &subtype, &(vsp::common::vs->to_vsp), sizeof(vsp::common::vs->to_vsp), MESSIP_NOTIMEOUT);
	
			if (rcvid == -1) /* Error condition, exit */
			{
				perror("VSP: Receive failed\n");
				break;
			} else if (rcvid < -1) {
				// ie. MESSIP_MSG_DISCONNECT
				fprintf(stderr, "VSP: ie. MESSIP_MSG_DISCONNECT\n");
				continue;
			}
	
			vsp::common::vs->from_vsp.vsp_report= lib::VSP_REPLY_OK;
	
			try {
				switch(vsp::common::vs->to_vsp.i_code) {
					case lib::VSP_CONFIGURE_SENSOR :
						vsp::common::vs->configure_sensor();
						break;
					case lib::VSP_INITIATE_READING :
						vsp::common::vs->initiate_reading();
						break;
					case lib::VSP_GET_READING :
						vsp::common::vs->get_reading();
						break;
					case lib::VSP_TERMINATE :
						vsp::common::vs->terminate();
						vsp::common::TERMINATE=true;
						break;
					default :
						throw lib::VSP_main_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
				}
			}

			catch (lib::VSP_main_error e){
				vsp::common::error_handler(e);
			} // end CATCH
			catch (lib::sensor::sensor_error e){
				vsp::common::error_handler(e);
			} // end CATCH
		
			messip_reply(ch, rcvid, 0, &vsp::common::vs->from_vsp, sizeof(lib::VSP_REPORT) + vsp::common::vs->union_size, MESSIP_NOTIMEOUT);
 		} // end while()
		vsp::common::vs->sr_msg->message ("VSP terminated");
	} // koniec TRY
	catch (lib::VSP_main_error e) {
		vsp::common::error_handler(e);
		exit(EXIT_FAILURE);
	} // end CATCH
} // end MAIN
