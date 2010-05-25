// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
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
#include <string.h>
#include <signal.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>

#include <fstream>						// do sprawdzenia czy istnieje plik /dev/TWOJSENSOR
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "lib/srlib.h"
#include "vsp/common/vsp_sensor.h"				// zawiera deklaracje klasy vsp_sensor + struktury komunikacyjne
// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace vsp {
namespace common {

/********************************* GLOBALS **********************************/
static sensor::sensor_interface *vs;		// czujnik wirtualny

static lib::condition_synchroniser vsp_synchroniser;

static bool TERMINATE = false; // zakonczenie obu watkow

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; // inicjalizacja MUTEXa

/********************************** SIGCATCH ********************************/
void catch_signal(int sig)
{
	switch (sig)
	{
		case SIGTERM:
			TERMINATE = true;
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in VSP process\n");
			signal(SIGSEGV, SIG_DFL);
			break;
	} // end: switch
}

/******************************** PROTOTYPES ********************************/
int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb);
int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb);
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb);

/***************************** ERROR_HANDLER ******************************/
template <class ERROR>
void error_handler(ERROR & e)
{
	switch (e.error_class)
	{
		case lib::SYSTEM_ERROR:
			if (e.error_no == DISPATCH_ALLOCATION_ERROR)
				printf("ERROR: Unable to allocate dispatch handle.\n");
			if (e.error_no == DEVICE_EXISTS)
				printf("ERROR: Device using that name already exists.\n");
			if (e.error_no == DEVICE_CREATION_ERROR)
				printf("ERROR: Unable to attach sensor device.\n");
			if (e.error_no == DISPATCH_LOOP_ERROR)
				printf("ERROR: Block error in main dispatch loop.\n");
			printf("VSP aborted due to lib::SYSTEM_ERROR\n");
			vsp::common::vs->sr_msg->message(lib::SYSTEM_ERROR, e.error_no);
			TERMINATE = true;
			break;
		case lib::FATAL_ERROR:
			vsp::common::vs->sr_msg->message(lib::FATAL_ERROR, e.error_no);
			break;
		case lib::NON_FATAL_ERROR:
			switch(e.error_no) {
			case INVALID_COMMAND_TO_VSP:
				vsp::common::vs->set_vsp_report(lib::INVALID_VSP_COMMAND);
				vsp::common::vs->sr_msg->message (lib::NON_FATAL_ERROR, e.error_no);
			break;
			case SENSOR_NOT_CONFIGURED:
				vsp::common::vs->set_vsp_report(lib::VSP_SENSOR_NOT_CONFIGURED);
				vsp::common::vs->sr_msg->message (lib::NON_FATAL_ERROR, e.error_no);
				break;
			case READING_NOT_READY:
				vsp::common::vs->set_vsp_report(lib::VSP_READING_NOT_READY);
				break;
			default:
				vsp::common::vs->sr_msg->message (lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
			}
			break;
		default:
			vsp::common::vs->sr_msg->message(lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
	} // end switch
} // end error_handler

/****************************** SECOND THREAD ******************************/
void* cyclic_read( void*  arg ){
	// ustawienie priorytetow
	setprio(0, MAX_PRIORITY-5);

	// aktywne oczekiwanie na zainicjowanie czujnika
	vsp_synchroniser.wait();

	while(!TERMINATE) { // for (;;)
		try{
			// oczekiwanie na zdarzenie
			vsp::common::vs->wait_for_event();
			// SEKCJA KRYTYCZNA - nie moze byc naraz odczyt z urzadzenia i zapis nowego odczytu
			pthread_mutex_lock( &mutex );
			vsp::common::vs->initiate_reading();
			pthread_mutex_unlock( &mutex );
			} // koniec TRY
		catch (vsp::sensor::VSP_main_error & e){
			error_handler(e);
			pthread_mutex_unlock( &mutex );
		} // end CATCH
		catch (lib::sensor::sensor_error & e){
			error_handler(e);
			pthread_mutex_unlock( &mutex );
		}
	}	// end for(;;)
	return(0);
}

/**************************** WRITE_TO_SENSOR ******************************/
void write_to_sensor() {
	lib::VSP_COMMAND_t i_code = vsp::common::vs->get_command();
	vsp::common::vs->set_vsp_report(lib::VSP_REPLY_OK);
	switch(i_code){
		case lib::VSP_CONFIGURE_SENSOR :
			vsp::common::vs->configure_sensor();
			vsp_synchroniser.command();
			break;
		case lib::VSP_GET_READING :
			vsp::common::vs->get_reading();
			break;
		case lib::VSP_TERMINATE :
			delete vsp::common::vs;
			TERMINATE=true;
			break;
		default :
			throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
	}
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
			vsp::common::vs->set_vsp_report(lib::VSP_REPLY_OK);
			vsp::common::vs->get_reading();
			} // end TRY
		catch (vsp::sensor::VSP_main_error & e){
			error_handler(e);
			} // end CATCH
		catch (lib::sensor::sensor_error & e){
			error_handler(e);
			} // end CATCH
       vs->msgwrite(ctp);
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
	vs->msgread(ctp);
	pthread_mutex_lock( &mutex );
		try{
		  write_to_sensor();
		  } // end TRY
		catch (vsp::sensor::VSP_main_error & e){
		  error_handler(e);
		  } // end CATCH
		catch (lib::sensor::sensor_error & e){
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
	vs->msgread(ctp);

    switch (msg->i.dcmd) {
    case DEVCTL_WT:
		pthread_mutex_lock( &mutex );
			try{
			  write_to_sensor();
			  } // end TRY
			catch (vsp::sensor::VSP_main_error & e){
			  error_handler(e);
			  } // end CATCH
			catch (lib::sensor::sensor_error & e){
				error_handler(e);
				} // end CATCH
		pthread_mutex_unlock( &mutex );
		return(EOK);
        break;
    case DEVCTL_RD:
		pthread_mutex_lock( &mutex );
			try{
				vsp::common::vs->set_vsp_report(lib::VSP_REPLY_OK);
				vsp::common::vs->get_reading();
				} // end TRY
			catch (vsp::sensor::VSP_main_error & e){
				error_handler(e);
				} // end CATCH
			catch (lib::sensor::sensor_error & e){
				error_handler(e);
				} // end CATCH
		  // Count the start address of reply message content.
		  vs->msgwrite(ctp);
		pthread_mutex_unlock( &mutex );
		return(EOK);
        break;
    case DEVCTL_RW:
		pthread_mutex_lock( &mutex );
			try{
				write_to_sensor();
				} // end TRY
			catch (vsp::sensor::VSP_main_error & e){
				error_handler(e);
				} // end CATCH
			catch (lib::sensor::sensor_error & e){
				error_handler(e);
				} // end CATCH
		  vs->msgwrite(ctp);
		pthread_mutex_unlock( &mutex );
		return(EOK);
        break;
    default:
        return(ENOSYS);
    }
	return(EOK);
}

} // namespace common
} // namespace vsp
} // namespace mrrocpp


/*********************************** MAIN ***********************************/
int main(int argc, char *argv[]) {

    /* declare variables we'll be using */
    resmgr_attr_t        resmgr_attr;
    dispatch_t           *dpp;
    dispatch_context_t   *ctp;
    int                  id;
	std::string resourceman_attach_point;
	static resmgr_connect_funcs_t   connect_funcs;
	static resmgr_io_funcs_t        io_funcs;
	static iofunc_attr_t            attr;

	// ustawienie priorytetow
	setprio(0, MAX_PRIORITY-1);
	// wylapywanie sygnalow
	signal(SIGTERM, &vsp::common::catch_signal);
	signal(SIGSEGV, &vsp::common::catch_signal);
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif

	// liczba argumentow
	if(argc < 6){
		printf("Za malo argumentow VSP\n");
		return -1;
		}

	 // zczytanie konfiguracji calego systemu
	lib::configurator _config(argv[1], argv[2], argv[3], argv[4], argv[5]);

	resourceman_attach_point = _config.return_attach_point_name(lib::configurator::CONFIG_RESOURCEMAN_LOCAL, "resourceman_attach_point");

	try{

 		// Stworzenie nowego czujnika za pomoca funkcji (cos na ksztalt szablonu abstract factory).
		vsp::common::vs = vsp::sensor::return_created_sensor(_config);

		// Sprawdzenie czy istnieje /dev/TWOJSENSOR.

		if( access(resourceman_attach_point.c_str(), R_OK)== 0 ){
			throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DEVICE_EXISTS);	// wyrzucany blad
		}

		/* initialize dispatch interface */
		if((dpp = dispatch_create()) == NULL)
			throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DISPATCH_ALLOCATION_ERROR);	// wyrzucany blad

		/* initialize resource manager attributes */
		memset(&resmgr_attr, 0, sizeof resmgr_attr);
		resmgr_attr.nparts_max = 1;
		resmgr_attr.msg_max_size = sizeof(mrrocpp::vsp::sensor::DEVCTL_MSG);

		/* initialize functions for handling messages */
		iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS, &io_funcs);
		io_funcs.read = vsp::common::io_read;
		io_funcs.write = vsp::common::io_write;
		io_funcs.devctl = vsp::common::io_devctl;

		/* initialize attribute structure used by the device */
		iofunc_attr_init(&attr, S_IFNAM | 0666, 0, 0);
		attr.nbytes = sizeof(mrrocpp::vsp::sensor::DEVCTL_MSG);


		/* attach our device name */
		if ( (id = resmgr_attach
				   (dpp,					/* dispatch handle        */
                       &resmgr_attr,			/* resource manager attrs */
                       resourceman_attach_point.c_str(),				/* device name            */
                       _FTYPE_ANY,			/* open type              */
                       0,						 /* flags                  */
                       &connect_funcs,		/* connect routines       */
                       &io_funcs,				/* I/O routines           */
                       &attr)) 	== -1){		/* handle                 */
			throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DEVICE_CREATION_ERROR);	// wyrzucany blad
			}

		/* allocate a context structure */
	 	ctp = dispatch_context_alloc(dpp);

		/* uruchomienie drugiego watku */
		pthread_attr_t tattr;
		pthread_attr_init( &tattr );
		pthread_attr_setdetachstate( &tattr, PTHREAD_CREATE_DETACHED );
		pthread_create( NULL, &tattr, &vsp::common::cyclic_read, NULL );

		/* start the resource manager message loop */
		while(!vsp::common::TERMINATE) { // for (;;)
			if((ctp = dispatch_block(ctp)) == NULL)
				throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DISPATCH_LOOP_ERROR);	// wyrzucany blad
			dispatch_handler(ctp);
		} // end for(;;)
	     vsp::common::vs->sr_msg->message ("VSP terminated");
	     delete vsp::common::vs;
		} // koniec TRY
	catch (vsp::sensor::VSP_main_error & e){
		vsp::common::error_handler(e);
		exit(EXIT_FAILURE);
	} // end CATCH
}	// end MAIN


