// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (VSP)
// Plik:            vsp_m_nint.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Nieinteraktywna powloka procesow VSP
//
// 	-	nieinteraktywne odczytywanie stanu czujnika rzeczywistego
// 	-	operacje read-write + devctl->(write, read, rw)
// 	-	dwuwatkowy
//
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------


/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <devctl.h>			// do devctl()
#include <semaphore.h>

#include <string.h>
#include <signal.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>

#include <fstream>						// do sprawdzenia czy istnieje plik /dev/TWOJSENSOR

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_sensor.h"				// zawiera deklaracje klasy vsp_sensor + struktury komunikacyjne

// Konfigurator
#include "lib/configurator.h"


/********************************* GLOBALS **********************************/
vsp_sensor *vs;		// czujnik wirtualny

sr_vsp *sr_msg;		// komunikacja z SR

sem_t start_sem;

configurator* config;

bool TERMINATE=false;											// zakonczenie obu watkow

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;	// inicjalizacja MUTEXa

/********************************** SIGCATCH ********************************/
void catch_signal(int sig) {
  switch(sig) {
	case SIGTERM :
	  TERMINATE = true;
	  vs->terminate();
	  sr_msg->message ("VSP terminated");
	  _exit(EXIT_SUCCESS);
	  break;
	case SIGSEGV:
	  fprintf(stderr, "Segmentation fault in VSP process\n");
	  signal(SIGSEGV, SIG_DFL);
	  break;
  } // end: switch
 }

/******************************** PROTOTYPES ********************************/
int io_read (resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb);
int io_write (resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb);
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb);

/***************************** ERROR_HANDLER ******************************/
template<class ERROR>
void error_handler(ERROR e){
	switch(e.error_class){
		case SYSTEM_ERROR:
			if(e.error_no == DISPATCH_ALLOCATION_ERROR)
				printf("ERROR: Unable to allocate dispatch handle.\n");
			if(e.error_no == DEVICE_EXISTS)
				printf("ERROR: Device using that name already exists.\n");
			if(e.error_no == DEVICE_CREATION_ERROR)
				printf("ERROR: Unable to attach sensor device.\n");
			if(e.error_no == DISPATCH_LOOP_ERROR)
				printf("ERROR: Block error in main dispatch loop.\n");
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

/****************************** SECOND THREAD ******************************/
void* cyclic_read( void*  arg ){
	// ustawienie priorytetow
	setprio(0, MAX_PRIORITY-5);

	// aktywne oczekiwanie na zainicjowanie czujnika
	sem_wait( &(start_sem));


	while(!TERMINATE) { // for (;;)
		try{
			// oczekiwanie na zdarzenie
			vs->wait_for_event();
			// SEKCJA KRYTYCZNA - nie moze byc naraz odczyt z urzadzenia i zapis nowego odczytu
			pthread_mutex_lock( &mutex );
			vs->initiate_reading();
			pthread_mutex_unlock( &mutex );
			} // koniec TRY
		catch (VSP_main_error e){
			error_handler(e);
			pthread_mutex_unlock( &mutex );
			} // end CATCH
		catch (sensor::sensor_error e){
			error_handler(e);
			pthread_mutex_unlock( &mutex );
			}; // end CATCH
		}	// end for(;;)
	return(0);
	}

/*********************************** MAIN ***********************************/
int main(int argc, char *argv[]) {

    /* declare variables we'll be using */
    resmgr_attr_t        resmgr_attr;
    dispatch_t           *dpp;
    dispatch_context_t   *ctp;
    int                  id;
    	char* resourceman_attach_point;
    	static resmgr_connect_funcs_t   connect_funcs;
	static resmgr_io_funcs_t        io_funcs;
	static iofunc_attr_t            attr;

    sem_init( &(start_sem), 0, 0);

	// ustawienie priorytetow
	setprio(0, MAX_PRIORITY-1);
	// wylapywanie sygnalow
	signal(SIGTERM, &catch_signal);
	signal(SIGSEGV, &catch_signal);

	// liczba argumentow
	if(argc < 6){
		printf("Za malo argumentow VSP\n");
		return -1;
		};

	 // zczytanie konfiguracji calego systemu
 	config = new configurator(argv[1], argv[2], argv[3], argv[4], argv[5]);
	if (argc>6) {
 		config->answer_to_y_rsh_spawn(argv[6]);
 		signal(SIGINT, SIG_IGN);
 	}
	resourceman_attach_point = config->return_attach_point_name(configurator::CONFIG_RESOURCEMAN_LOCAL, "resourceman_attach_point");

	try{

 		/* Lokalizacja procesu wyswietlania komunikatow SR */
		if ((sr_msg = new sr_vsp(VSP, config->return_string_value("resourceman_attach_point"),
			 config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]"))) == NULL)
		{
				printf("communication with SR not ready\n");
		}
		else
			sr_msg->message ("Communication with SR ready");

		// Stworzenie nowego czujnika za pomoca funkcji (cos na ksztalt szablonu abstract factory).
		vs = return_created_sensor();

		// Sprawdzenie czy istnieje /dev/TWOJSENSOR.

		if( access(resourceman_attach_point, R_OK)== 0 ){
			throw VSP_main_error(SYSTEM_ERROR, DEVICE_EXISTS);	// wyrzucany blad
		};

		/* initialize dispatch interface */
		if((dpp = dispatch_create()) == NULL)
			throw VSP_main_error(SYSTEM_ERROR, DISPATCH_ALLOCATION_ERROR);	// wyrzucany blad

		/* initialize resource manager attributes */
		memset(&resmgr_attr, 0, sizeof resmgr_attr);
		resmgr_attr.nparts_max = 1;
		resmgr_attr.msg_max_size = sizeof(DEVCTL_MSG);

		/* initialize functions for handling messages */
		iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS, &io_funcs);
		io_funcs.read = io_read;
		io_funcs.write = io_write;
		io_funcs.devctl = io_devctl;

		/* initialize attribute structure used by the device */
		iofunc_attr_init(&attr, S_IFNAM | 0666, 0, 0);
		attr.nbytes = sizeof(DEVCTL_MSG);


		/* attach our device name */
		if ( (id = resmgr_attach
				   (dpp,					/* dispatch handle        */
                       &resmgr_attr,			/* resource manager attrs */
                       resourceman_attach_point,				/* device name            */
                       _FTYPE_ANY,			/* open type              */
                       0,						 /* flags                  */
                       &connect_funcs,		/* connect routines       */
                       &io_funcs,				/* I/O routines           */
                       &attr)) 	== -1){		/* handle                 */
			throw VSP_main_error(SYSTEM_ERROR, DEVICE_CREATION_ERROR);	// wyrzucany blad
			};

		/* allocate a context structure */
	 	ctp = dispatch_context_alloc(dpp);

		/* uruchomienie drugiego watku */
		pthread_attr_t tattr;
		pthread_attr_init( &tattr );
		pthread_attr_setdetachstate( &tattr, PTHREAD_CREATE_DETACHED );
		pthread_create( NULL, &tattr, &cyclic_read, NULL );

		/* start the resource manager message loop */
		while(!TERMINATE) { // for (;;)
			if((ctp = dispatch_block(ctp)) == NULL)
				throw VSP_main_error(SYSTEM_ERROR, DISPATCH_LOOP_ERROR);	// wyrzucany blad
			dispatch_handler(ctp);
	 		} // end for(;;)
	     sr_msg->message ("VSP terminated");
		} // koniec TRY
	catch (VSP_main_error e){
		error_handler(e);
		exit(EXIT_FAILURE);
		}; // end CATCH
	}	// end MAIN


/**************************** WRITE_TO_SENSOR ******************************/
void write_to_sensor(VSP_COMMAND i_code){
	vs->from_vsp.vsp_report=VSP_REPLY_OK;
	switch(i_code){
		case VSP_CONFIGURE_SENSOR :
			vs->configure_sensor();
			while (sem_trywait ( &(start_sem)) == 0);
			sem_post( &(start_sem));
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
		};
}

/********************************* IO_READ **********************************/
int io_read (resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb){
    int status;
	if ((status = iofunc_read_verify (ctp, msg, ocb, NULL)) != EOK)
		return (status);
	if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
		return (ENOSYS);
	// SEKCJA KRYTYCZNA - nie moze byc naraz odczyt z urzadzenia i zapis nowego odczytu
	pthread_mutex_lock( &mutex );
	  try{
			vs->from_vsp.vsp_report=VSP_REPLY_OK;
			vs->get_reading();
			} // end TRY
		catch (VSP_main_error e){
			error_handler(e);
			} // end CATCH
		catch (sensor::sensor_error e){
			error_handler(e);
			} // end CATCH
       resmgr_msgwrite(ctp, &vs->from_vsp, sizeof(VSP_REPORT) + vs->union_size, 0);
	pthread_mutex_unlock( &mutex );
	return(EOK);
}	// end io_read

/********************************* IO_WRITE *********************************/
int io_write (resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb){
    int     status;
	if ((status = iofunc_write_verify(ctp, msg, ocb, NULL)) != EOK)
		return (status);
	if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
		return(ENOSYS);
	_IO_SET_WRITE_NBYTES (ctp, msg->i.nbytes);
	resmgr_msgread(ctp, &vs->to_vsp, msg->i.nbytes, sizeof(msg->i));
	pthread_mutex_lock( &mutex );
		try{
		  write_to_sensor(vs->to_vsp.i_code);
		  } // end TRY
		catch (VSP_main_error e){
		  error_handler(e);
		  } // end CATCH
		catch (sensor::sensor_error e){
			error_handler(e);
			} // end CATCH
	pthread_mutex_unlock( &mutex );
	return (EOK);
} // end io_write


/******************************** IO_DEVCTL *********************************/
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb) {
    unsigned int status;
	int *addr;

    if ((status = iofunc_devctl_default(ctp, msg, ocb)) != _RESMGR_DEFAULT)
        return(status);

	_IO_SET_WRITE_NBYTES (ctp, msg->i.nbytes);
	resmgr_msgread(ctp, &vs->to_vsp, msg->i.nbytes, sizeof(msg->i));

    switch (msg->i.dcmd) {
    case DEVCTL_WT:
		pthread_mutex_lock( &mutex );
			try{
			  write_to_sensor(vs->to_vsp.i_code);
			  } // end TRY
			catch (VSP_main_error e){
			  error_handler(e);
			  } // end CATCH
			catch (sensor::sensor_error e){
				error_handler(e);
				} // end CATCH
		pthread_mutex_unlock( &mutex );
		return(EOK);
        break;
    case DEVCTL_RD:
		pthread_mutex_lock( &mutex );
			try{
				vs->from_vsp.vsp_report=VSP_REPLY_OK;
				vs->get_reading();
				} // end TRY
			catch (VSP_main_error e){
				error_handler(e);
				} // end CATCH
			catch (sensor::sensor_error e){
				error_handler(e);
				} // end CATCH
		  // Count the start address of reply message content.
		  /*
		  struct _io_devctl_reply {
	          uint32_t                  zero;
	          int32_t                   ret_val;
	          int32_t                   nbytes;
	          int32_t                   zero2;
		     // char                      data[nbytes];//
		  =>
		  &data = &_io_devctl_reply + 16bytes = &_io_devctl_reply + 4*int
		  */
		  addr = (int*)&msg->o +4;
		  memcpy( addr, &vs->from_vsp, vs->union_size + sizeof(VSP_REPORT));
		  resmgr_msgwrite(ctp, &msg->o,  sizeof(msg->o) + sizeof(VSP_REPORT) + vs->union_size, 0);
		pthread_mutex_unlock( &mutex );
		return(EOK);
        break;
    case DEVCTL_RW:
		pthread_mutex_lock( &mutex );
			try{
				write_to_sensor(vs->to_vsp.i_code);
				} // end TRY
			catch (VSP_main_error e){
				error_handler(e);
				} // end CATCH
			catch (sensor::sensor_error e){
				error_handler(e);
				} // end CATCH
		  // Count the start address of reply message content.
		  addr = (int*)&msg->o +4;
		  memcpy( addr, &vs->from_vsp, vs->union_size + sizeof(VSP_REPORT));
		  resmgr_msgwrite(ctp, &msg->o,  sizeof(msg->o) + sizeof(VSP_REPORT) + vs->union_size, 0);
		pthread_mutex_unlock( &mutex );
		return(EOK);
        break;
    default:
        return(ENOSYS);
    }
	return(EOK);

}
