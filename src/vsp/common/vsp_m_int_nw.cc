// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:            vsp_m_nint.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Interaktywna (bez oczekiwania) powloka procesow VSP
//
// 	-	interaktywne odczytywanie stanu czujnika rzeczywistego, bez oczekiwania na zakonczenie wynnosci
// 	-	operacje read-write + devctl->(write, read, rw)
// 	-	dwuwatkowy
//   -   wykorzystanie informacji CONFIGURE_FLAG oraz INITIATE_FLAG
//   -    poprawiony blad przy komunikacji za pomoca DevCtl.
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

#include <fstream>			// do sprawdzenia czy istnieje plik /dev/TWOJSENSOR
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
static sensor::sensor_interface *vs; // czujnik wirtualny

// Zwracane dane.
lib::VSP_ECP_MSG ret_msg;

static pthread_mutex_t image_mutex = PTHREAD_MUTEX_INITIALIZER; // inicjalizacja MUTEXa uzywanego przy synchronizacji zapisu i odczytu z obrazu

static lib::condition_synchroniser synchroniser;

static bool TERMINATE = false; // zakonczenie obu watkow
static bool CONFIGURE_FLAG = false; // czy skonfigurowany czujnik
static bool INITIATE_FLAG = false; // czy odczyt zainicjowany
static bool sensor_configuration_task = false; // nalezy wykonac konfiguracje sensora
static bool reading_initiation_task = false; // nalezy wykonac inicjacje odczytu

/********************************** SIGCATCH ********************************/
void catch_signal(int sig)
{
	switch (sig)
	{
		case SIGTERM:
			TERMINATE = true;
			synchroniser.command();
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
			vs->sr_msg->message(lib::SYSTEM_ERROR, e.error_no);
			TERMINATE = true;
			synchroniser.command();
			break;
		case lib::FATAL_ERROR:
			vs->sr_msg->message(lib::FATAL_ERROR, e.error_no);
			break;
		case lib::NON_FATAL_ERROR:
			switch (e.error_no)
			{
				case INVALID_COMMAND_TO_VSP:
					ret_msg.vsp_report = lib::INVALID_VSP_COMMAND;
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				case SENSOR_NOT_CONFIGURED:
					ret_msg.vsp_report = lib::VSP_SENSOR_NOT_CONFIGURED;
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				case READING_NOT_READY:
					ret_msg.vsp_report = lib::VSP_READING_NOT_READY;
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;

				default:
					vs->sr_msg->message(lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
			}
			break;
		default:
			vs->sr_msg->message(lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
	} // end switch
} // end error_handler

/****************************** SECOND THREAD ******************************/
void* execute_command(void* arg)
{
	// ustawienie priorytetow
	setprio(0, 1);
	while (!TERMINATE) { // for (;;)
		// aktywne oczekiwanie, az bedzie jakies zadanie do wykonania

		synchroniser.wait();

		// Zadanie konfiguracji czujnika.
		if (sensor_configuration_task) {
			CONFIGURE_FLAG = false;
			pthread_mutex_lock(&image_mutex);
			try {
				vs->configure_sensor();
			} // end TRY
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} // end CATCH
			pthread_mutex_unlock(&image_mutex);
			sensor_configuration_task = false;
			CONFIGURE_FLAG = true;
		}

		// Zadanie inicjacji odczytow.
		if (reading_initiation_task) {
			INITIATE_FLAG = false;
			pthread_mutex_lock(&image_mutex);
			try {
				vs->initiate_reading();
			} // end TRY
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} // end CATCH
			pthread_mutex_unlock(&image_mutex);
			reading_initiation_task = false;
			INITIATE_FLAG = true;
		}
	} // koniec for(;;)
	return (0);
}//: execute_command


/**************************** WRITE_TO_SENSOR ******************************/
void write_to_sensor()
{
	lib::VSP_COMMAND_t i_code = vs->get_command();

	//printf("VSP: write_to_sensor begin\n");
	switch (i_code)
	{
		case lib::VSP_CONFIGURE_SENSOR:
			ret_msg.vsp_report = lib::VSP_REPLY_OK;
			sensor_configuration_task = true;
			synchroniser.command();
			return;
		case lib::VSP_INITIATE_READING:
			ret_msg.vsp_report = lib::VSP_REPLY_OK;
			try {
				if (!CONFIGURE_FLAG)
					throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
				reading_initiation_task = true;
				synchroniser.command();
			} // end TRY
			catch (vsp::sensor::VSP_main_error & e) {
				error_handler(e);
			} // end CATCH
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} // end CATCH
			return;
		case lib::VSP_GET_READING:
			try {
				if (!CONFIGURE_FLAG)
					throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
				if (!INITIATE_FLAG)
					throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, READING_NOT_READY);
				// SEKCJA KRYTYCZNA - nie moze byc naraz odczyt z urzadzenia i zapis nowego odczytu
				pthread_mutex_lock(&image_mutex);
				vs->set_vsp_report(lib::VSP_REPLY_OK);
				vs->get_reading();
				// Przepisanie obrazu z czujnika do wiadomosci zwrotnej.
				ret_msg = vs->from_vsp;
				pthread_mutex_unlock(&image_mutex);
				INITIATE_FLAG = false;
			} // end TRY
			catch (vsp::sensor::VSP_main_error & e) {
				error_handler(e);
			} // end CATCH
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} // end CATCH
			return;
		case lib::VSP_TERMINATE:
			vs->set_vsp_report(lib::VSP_REPLY_OK);
			delete vs;
			TERMINATE = true;
			synchroniser.command();
			return;
		default:
			throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
	}
	//printf("VSP: write_to_sensor_end\n");
}

/********************************* IO_READ **********************************/
int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
	//printf("VSP: io_read begin\n");
	int status;
	if ((status = iofunc_read_verify(ctp, msg, ocb, NULL)) != EOK)
		return (status);
	if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
		return (ENOSYS);
	try {
		if (!CONFIGURE_FLAG)
			throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		if (!INITIATE_FLAG)
			throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, READING_NOT_READY);
		// SEKCJA KRYTYCZNA - nie moze byc naraz odczyt z urzadzenia i zapis nowego odczytu
		pthread_mutex_lock(&image_mutex);
		vs->set_vsp_report(lib::VSP_REPLY_OK);
		vs->get_reading();
		// Przepisanie obrazu z czujnika do wiadomosci zwrotnej.
		ret_msg = vs->from_vsp;
		pthread_mutex_unlock(&image_mutex);
		// Koniec sekcji krytycznej.
	} // end TRY
	catch (vsp::sensor::VSP_main_error & e) {
		error_handler(e);
		//		pthread_mutex_unlock( &image_mutex );
	} // end CATCH
	catch (lib::sensor::sensor_error & e) {
		error_handler(e);
		//		pthread_mutex_unlock( &image_mutex );
	} // end CATCH
	// Wyslanie odpowiedzi - wiadomosci wzrotnej.
	vs->msgwrite(ctp);
	INITIATE_FLAG = false;
	return (EOK);
	//printf("VSP: io_read end\n");
} // end io_read

/********************************* IO_WRITE *********************************/
int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)
{
	//printf("VSP: io_write begin\n");
	int status;
	if ((status = iofunc_write_verify(ctp, msg, ocb, NULL)) != EOK)
		return (status);
	if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
		return (ENOSYS);
	_IO_SET_WRITE_NBYTES (ctp, msg->i.nbytes);
	vs->msgread(ctp);
	write_to_sensor();
	return (EOK);
	printf("VSP: io_write end\n");
} // end io_write


/******************************** IO_DEVCTL *********************************/
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
	//printf("VSP: io_devctl end\n");
	unsigned int status;
	int *addr;
	if ((status = iofunc_devctl_default(ctp, msg, ocb)) != _RESMGR_DEFAULT)
		return (status);

	_IO_SET_WRITE_NBYTES (ctp, msg->i.nbytes);
	vs->msgread(ctp);

	switch (msg->i.dcmd)
	{
		case DEVCTL_WT:
			write_to_sensor();
			return (EOK);
			break;
		case DEVCTL_RD:
			try {
				if (!CONFIGURE_FLAG)
					throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
				if (!INITIATE_FLAG)
					throw vsp::sensor::VSP_main_error(lib::NON_FATAL_ERROR, READING_NOT_READY);
				// SEKCJA KRYTYCZNA - nie moze byc naraz odczyt z urzadzenia i zapis nowego odczytu
				pthread_mutex_lock(&image_mutex);
				vs->set_vsp_report(lib::VSP_REPLY_OK);
				vs->get_reading();
				// Przepisanie obrazu z czujnika do wiadomosci zwrotnej.
				ret_msg = vs->from_vsp;
				INITIATE_FLAG = false;
				pthread_mutex_unlock(&image_mutex);
				// Koniec sekcji krytycznej.
			} // end TRY
			catch (vsp::sensor::VSP_main_error & e) {
				error_handler(e);
			} // end CATCH
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} // end CATCH
			vs->msgwrite(ctp);
			return (EOK);
			break;
		case DEVCTL_RW:
			write_to_sensor();
			vs->msgwrite(ctp);
			return (EOK);
			break;
		default:
			return (ENOSYS);
	}
	return (EOK);
}

} // namespace common
} // namespace vsp
} // namespace mrrocpp


/*********************************** MAIN ***********************************/
int main(int argc, char *argv[])
{

	/* declare variables we'll be using */
	resmgr_attr_t resmgr_attr;
	dispatch_t *dpp;
	dispatch_context_t *ctp;
	int id;
	std::string resourceman_attach_point;

	static resmgr_connect_funcs_t connect_funcs;
	static resmgr_io_funcs_t io_funcs;
	static iofunc_attr_t attr;

	// wylapywanie sygnalow
	signal(SIGTERM, &vsp::common::catch_signal);
	signal(SIGSEGV, &vsp::common::catch_signal);
#if defined(PROCESS_SPAWN_RSH)
	signal(SIGINT, SIG_IGN);
#endif

	// liczba argumentow
	if (argc < 6) {
		printf("Za malo argumentow VSP\n");
		return -1;
	}

	// zczytanie konfiguracji calego systemu
	lib::configurator _config(argv[1], argv[2], argv[3], argv[4], argv[5]);

	resourceman_attach_point
			= _config.return_attach_point_name(lib::configurator::CONFIG_RESOURCEMAN_LOCAL, "resourceman_attach_point");

	try {

		// Stworzenie nowego czujnika za pomoca funkcji (cos na ksztalt szablonu abstract factory).
		vsp::common::vs = vsp::sensor::return_created_sensor(_config);

		// Sprawdzenie czy istnieje /dev/TWOJSENSOR.
		if (access(resourceman_attach_point.c_str(), R_OK) == 0) {

			throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DEVICE_EXISTS); // wyrzucany blad
		}

		/* initialize dispatch interface */
		if ((dpp = dispatch_create()) == NULL)
			throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DISPATCH_ALLOCATION_ERROR); // wyrzucany blad

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
		if ((id = resmgr_attach(dpp, /* dispatch handle        */
		&resmgr_attr, /* resource manager attrs */
		resourceman_attach_point.c_str(), /* device name            */
		_FTYPE_ANY, /* open type              */
		0, /* flags                  */
		&connect_funcs, /* connect routines       */
		&io_funcs, /* I/O routines           */
		&attr)) == -1) { /* handle                 */
			throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DEVICE_CREATION_ERROR); // wyrzucany blad
		}

		/* allocate a context structure */
		ctp = dispatch_context_alloc(dpp);

		/* uruchomienie drugiego watku */
		pthread_attr_t tattr;
		pthread_attr_init(&tattr);
		pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED);
		pthread_attr_setschedpolicy(&tattr, SCHED_RR);
		pthread_create(NULL, &tattr, &vsp::common::execute_command, NULL);

		// ustawienie priorytetow
		setprio(0, 15);
		flushall();
		/* start the resource manager message loop */
		vsp::common::vs->sr_msg->message("Device is waiting for clients...");
		while (!vsp::common::TERMINATE) { // for(;;)
		//printf("VSP: main loop begin\n");
			if ((ctp = dispatch_block(ctp)) == NULL)
				throw vsp::sensor::VSP_main_error(lib::SYSTEM_ERROR, DISPATCH_LOOP_ERROR); // wyrzucany blad
			dispatch_handler(ctp);
			//printf("VSP: main loop end\n");
		} // end for(;;)
		vsp::common::vs->sr_msg->message("VSP terminated");
		delete vsp::common::vs;
	} // koniec TRY
	catch (vsp::sensor::VSP_main_error & e) {
		vsp::common::error_handler(e);
		exit(EXIT_FAILURE);
	}
} // end MAIN
