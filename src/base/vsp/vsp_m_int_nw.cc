/*!
 \file vsp_m_int_nw.cc

 \brief file contains the interactive VSP shell

 The interactive VSP shell should be used for all sensors,
 that are required to cooperate with MP/ECP processes interativelly, which means that they should perform reading initiation and aggregation to the form useful in control only when such command will be received.

The nw stands for NO WAIT, which means that the VSP sends response right after the command was received and the command is in fact being executed separatedly - in another thread.

 \date 30.11.2006
 \author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology

 \defgroup VSP -- Virtual Sensor Process
 */

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <errno.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <devctl.h>
#include <string.h>
#include <signal.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "lib/srlib.h"
#include "lib/configurator.h"
#include "base/vsp/vsp_sensor.h"
#include "base/vsp/vsp_error.h"

namespace mrrocpp {
namespace vsp {
namespace common {

/** Global pointer to sensor object. */
static sensor::sensor_interface *vs;

/** Returned message. */
lib::VSP_ECP_MSG ret_msg;

/** Mutex utilized for read/write sensor image synchronization. */
static pthread_mutex_t image_mutex = PTHREAD_MUTEX_INITIALIZER;

/**  Condition synchronizer. */
static lib::condition_synchroniser synchroniser;

/** Threads termination flag. */
static bool TERMINATE = false;

/** Sensor configured flag. */
static bool CONFIGURE_FLAG = false;

/** Reading initialized flag. */
static bool INITIATE_FLAG = false;

/** Flag used for passing the sensor configuration command. */
static bool sensor_configuration_task = false;

/** Flag used for passing the reading initiation command. */
static bool reading_initiation_task = false;

/**
 * @brief Signal handler.
 * @author tkornuta
 * @param sig Catched signal
 */
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

/*int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb);
int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb);
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb);
*/

/**
 * @brief Function responsible for handling errors.
 * @author tkornuta
 * @param e Handled error
 */
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
			printf("vsp  aborted due to lib::SYSTEM_ERROR\n");
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
	} //: switch
} //: error_handler


/**
 * @brief Body of the thread responsible for commands execution.
 * @author tkornuta
 * @param arg Thread arguments - not used
 */
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
			} //: try
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
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
			} //: try
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
			pthread_mutex_unlock(&image_mutex);
			reading_initiation_task = false;
			INITIATE_FLAG = true;
		}
	} //: for(;;)
	return (0);
}//: execute_command


/**
 * @brief Function responsible for parsing the command send by the ECP/MP and - in case of GET_READING - copies current reading to returned message buffer.
 * @author tkornuta
 */
void parse_command()
{
	lib::VSP_COMMAND_t i_code = vs->get_command();

	//printf("vsp: write_to_sensor begin\n");
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
					throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
				reading_initiation_task = true;
				synchroniser.command();
			} //: try
			catch (vsp::sensor::vsp_error & e) {
				error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
			return;
		case lib::VSP_GET_READING:
			try {
				if (!CONFIGURE_FLAG)
					throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
				if (!INITIATE_FLAG)
					throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, READING_NOT_READY);
				// Critical section.
				pthread_mutex_lock(&image_mutex);
				vs->set_vsp_report(lib::VSP_REPLY_OK);
				vs->get_reading();
				// Copy current sensory data to output buffer.
				ret_msg = vs->from_vsp;
				pthread_mutex_unlock(&image_mutex);
				INITIATE_FLAG = false;
			} //: try
			catch (vsp::sensor::vsp_error & e) {
				error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				e end rror_handler(e);
			} //: catch
			return;
		case lib::VSP_TERMINATE:
			vs->set_vsp_report(lib::VSP_REPLY_OK);
			delete vs;
			TERMINATE = true;
			synchroniser.command();
			return;
		default:
			throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
	}
	//printf("vsp: write_to_sensor_end\n");
}

/**
 * @brief The io_read handler is responsible for returning data bytes to the client after receiving an _IO_READ message.
 * @author tkornuta
 * @param ctp Resource manager context
 * @param msg Returned message
 * @param ocb Position within the buffer that will be returned to the client
 * @return Status of the operation
 */
int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
	//printf("vsp: io_read begin\n");
	int status;
	if ((status = iofunc_read_verify(ctp, msg, ocb, NULL)) != EOK)
		return (status);
	if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
		return (ENOSYS);
	try {
		if (!CONFIGURE_FLAG)
			throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		if (!INITIATE_FLAG)
			throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, READING_NOT_READY);
		// Critical section.
		pthread_mutex_lock(&image_mutex);
		vs->set_vsp_report(lib::VSP_REPLY_OK);
		vs->get_reading();
		// Copy current sensory data to output buffer.
		ret_msg = vs->from_vsp;
		pthread_mutex_unlock(&image_mutex);
		//: Critical section.
	} //: try
	catch (vsp::sensor::vsp_error & e) {
		error_handler(e);
	} //: catch
	catch (lib::sensor::sensor_error & e) {
		error_handler(e);
	} //: catch
	// Write response to context.
	vs->msgwrite(ctp);
	INITIATE_FLAG = false;
	return (EOK);
} //: io_read

/**
 * @brief The io_write handler is responsible for writing data bytes to the media after receiving a client's _IO_WRITE message.
 * @author tkornuta
 * @param ctp Resource manager context
 * @param msg Retrieved message
 * @param ocb Position within the buffer retrieved from the client
 * @return Status of the operation
 */
int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)
{
	int status;
	// Check operation status.
	if ((status = iofunc_write_verify(ctp, msg, ocb, NULL)) != EOK)
		return (status);
	// Check message type.
	if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
		return (ENOSYS);
	// Set up the number of bytes (returned by client's write()).
	_IO_SET_WRITE_NBYTES(ctp, msg->i.nbytes);
	// Read message from the context.
	vs->msgread(ctp);
	// Parse retrieved command.
	parse_command();
	return (EOK);
} //: io_write


/**
 * @brief Handler function to handle the _IO_DEVCTL message.
 * @author tkornuta
 * @param ctp Resource manager context
 * @param msg Retrieved/send back message
 * @param ocb Position within the buffer retrieved from/send back to the client
 * @return Status of the operation
 */
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
	unsigned int status;
	int *addr;
	// Check operation status.
	if ((status = iofunc_devctl_default(ctp, msg, ocb)) != _RESMGR_DEFAULT)
		return (status);

	// Set up the number of bytes (returned by client's write()).
	_IO_SET_WRITE_NBYTES(ctp, msg->i.nbytes);
	// Read message from the context.
	vs->msgread(ctp);

	// Check devctl operation type.
	switch (msg->i.dcmd)
	{
		case DEVCTL_WT:
			parse_command();
			return (EOK);
			break;
		case DEVCTL_RD:
			try {
				if (!CONFIGURE_FLAG)
					throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, SENSOR_NOT_CONFIGURED);
				if (!INITIATE_FLAG)
					throw vsp::sensor::vsp_error(lib::NON_FATAL_ERROR, READING_NOT_READY);
				// Critical section.
				pthread_mutex_lock(&image_mutex);
				vs->set_vsp_report(lib::VSP_REPLY_OK);
				vs->get_reading();
				// Copy current sensory data to output buffer.
				ret_msg = vs->from_vsp;
				INITIATE_FLAG = false;
				pthread_mutex_unlock(&image_mutex);
				//: Critical section.
			} //: try
			catch (vsp::sensor::vsp_error & e) {
				error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
			vs->msgwrite(ctp);
			return (EOK);
			break;
		case DEVCTL_RW:
			parse_command();
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

			throw vsp::sensor::vsp_error(lib::SYSTEM_ERROR, DEVICE_EXISTS); // wyrzucany blad
		}

		/* initialize dispatch interface */
		if ((dpp = dispatch_create()) == NULL)
			throw vsp::sensor::vsp_error(lib::SYSTEM_ERROR, DISPATCH_ALLOCATION_ERROR); // wyrzucany blad

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
			throw vsp::sensor::vsp_error(lib::SYSTEM_ERROR, DEVICE_CREATION_ERROR); // wyrzucany blad
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
			//printf("vsp: main loop begin\n");
			if ((ctp = dispatch_block(ctp)) == NULL)
				throw vsp::sensor::vsp_error(lib::SYSTEM_ERROR, DISPATCH_LOOP_ERROR); // wyrzucany blad
			dispatch_handler(ctp);
			//printf("vsp: main loop end\n");
		} //: for(;;)
		vsp::common::vs->sr_msg->message("vsp  terminated");
		delete vsp::common::vs;
	} //: try
	catch (vsp::sensor::vsp_error & e) {
		vsp::common::error_handler(e);
		exit(EXIT_FAILURE);
	}
} //: main
