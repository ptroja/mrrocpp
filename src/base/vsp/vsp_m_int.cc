/*!
 * @file
 * @brief File containing the \b interactive VSP shell.
 *
 * The \b interactive VSP shell collects new measurements only when an explicit reading initiation request arrives.
 * In case of this shell the higher layer processes must wait until the VSP finishes ordered task.
 *
 * @date 30.11.2006
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup VSP
 */

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <cerrno>
#include <cstddef>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <devctl.h>
#include <cstring>
#include <csignal>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <fstream>

// niezbedny naglowek z definiacja PROCESS_SPAWN_RSH
#include "base/lib/configurator.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/sr/srlib.h"
#include "base/vsp/vsp_sensor_interface.h"
#include "base/vsp/vsp_error.h"

namespace mrrocpp {
namespace vsp {
namespace int_shell {

/** @brief Global pointer to sensor object. */
static mrrocpp::vsp::common::sensor_interface *vs;

/** @brief Threads termination flag. */
static bool TERMINATED = false;


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
			TERMINATED = true;
			break;
		case lib::FATAL_ERROR:
			vs->sr_msg->message(lib::FATAL_ERROR, e.error_no);
			break;
		case lib::NON_FATAL_ERROR:
			switch (e.error_no)
			{
				case INVALID_COMMAND_TO_VSP:
					vs->set_vsp_report(lib::sensor::INVALID_VSP_COMMAND);
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				case SENSOR_NOT_CONFIGURED:
					vs->set_vsp_report(lib::sensor::VSP_SENSOR_NOT_CONFIGURED);
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				case READING_NOT_READY:
					vs->set_vsp_report(lib::sensor::VSP_READING_NOT_READY);
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
 * @brief Signal handler. Sets terminate flag in case of the SIGTERM signal.
 * @author tkornuta
 * @param sig Catched signal.
 */
void catch_signal(int sig)
{
	switch (sig)
	{
		case SIGTERM:
			TERMINATED = true;
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in the VSP process\n");
			signal(SIGSEGV, SIG_DFL);
			break;
	} // end: switch
}


/**
 * @brief Function responsible for parsing the command send by the ECP/MP and - in case of GET_READING - copies current reading to returned message buffer.
 * @author tkornuta
 */
void parse_command(void)
{
	lib::sensor::VSP_COMMAND_t i_code = vs->get_command();
	vs->set_vsp_report(lib::sensor::VSP_REPLY_OK);

	switch (i_code)
	{
		case lib::sensor::VSP_CONFIGURE_SENSOR:
			vs->configure_sensor();
			break;
		case lib::sensor::VSP_INITIATE_READING:
			vs->initiate_reading();
			break;
		case lib::sensor::VSP_GET_READING:
			vs->get_reading();
			break;
		case lib::sensor::VSP_TERMINATE:
			TERMINATED = true;
			break;
		default:
			throw vsp::common::vsp_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
	}
}


/**
 * @brief The io_read handler is responsible for returning data bytes to the client after receiving an _IO_READ message.
 * @author tkornuta
 * @param ctp Resource manager context.
 * @param msg Returned message.
 * @param ocb Position within the buffer that will be returned to the client.
 * @return Status of the operation.
 */
int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
	int status;
	// Check operation status.
	if ((status = iofunc_read_verify(ctp, msg, ocb, NULL)) != EOK) {
		return (status);
	}
	if (msg->i.xtype & (_IO_XTYPE_MASK != _IO_XTYPE_NONE)) {
		return (ENOSYS);
	}
	// Critical section.
	try {
		vs->set_vsp_report(lib::sensor::VSP_REPLY_OK);
		vs->get_reading();
	} //: try
	catch (vsp::common::vsp_error & e) {
		error_handler(e);
	} //: catch
	catch (lib::sensor::sensor_error & e) {
		error_handler(e);
	} //: catch
	// Write response to context.
	vs->msgwrite(ctp);
	return (EOK);
} //: io_read


/**
 * @brief The io_write handler is responsible for writing data bytes to the media after receiving a client's _IO_WRITE message.
 * @author tkornuta
 * @param ctp Resource manager context.
 * @param msg Retrieved message.
 * @param ocb Position within the buffer retrieved from the client.
 * @return Status of the operation.
 */
int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)
{
	int status;
	// Check operation status.
	if ((status = iofunc_write_verify(ctp, msg, ocb, NULL)) != EOK) {
		return (status);
	}
	// Check message type.
	if (msg->i.xtype & (_IO_XTYPE_MASK != _IO_XTYPE_NONE)) {
		return (ENOSYS);
	}
	// Set up the number of bytes (returned by client's write()).
	_IO_SET_WRITE_NBYTES(ctp, msg->i.nbytes);
	// Read message from the context.
	vs->msgread(ctp);
	try {
		parse_command();
	} //: try
	catch (vsp::common::vsp_error & e) {
		error_handler(e);
	} //: catch
	catch (lib::sensor::sensor_error & e) {
		error_handler(e);
	} //: catch
	return (EOK);
} //: io_write


/**
 * @brief Handler function for handling the _IO_DEVCTL messages.
 * @author tkornuta
 * @param ctp Resource manager context.
 * @param msg Retrieved/send back message.
 * @param ocb Position within the buffer retrieved from/send back to the client.
 * @return Status of the operation.
 */
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
	unsigned int status;
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
			try {
				parse_command();
			} //: try
			catch (vsp::common::vsp_error & e) {
				error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
			return (EOK);
			break;
		case DEVCTL_RD:
			try {
				vs->set_vsp_report(lib::sensor::VSP_REPLY_OK);
				vs->get_reading();
			} //: try
			catch (vsp::common::vsp_error & e) {
				error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
			// Write response to the resource manager context..
			vs->msgwrite(ctp);
			return (EOK);
			break;
		case DEVCTL_RW:
			try {
				parse_command();
			} //: try
			catch (vsp::common::vsp_error & e) {
				error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				error_handler(e);
			} //: catch
			// Write response to the resource manager context..
			vs->msgwrite(ctp);
			return (EOK);
			break;
		default:
			return (ENOSYS);
	}
	return (EOK);
}

} // namespace int_shell
} // namespace vsp
} // namespace mrrocpp


/**
 * @brief Main body of the interactive VSP shell.
 * @param argc Number of passed arguments.
 * @param argv Process arguments - network node, path to the MRROC++ binaries, name of the current configuration file, etc.
 * @return Process status.
 */
int main(int argc, char *argv[])
{
	// Declare variables we'll be using.
	resmgr_attr_t resmgr_attr;
	dispatch_t *dpp;
	dispatch_context_t *ctp;
	int id;
	std::string resourceman_attach_point;

	static resmgr_connect_funcs_t connect_funcs;
	static resmgr_io_funcs_t io_funcs;
	static iofunc_attr_t attr;

	// Set priority.
	setprio(getpid(), lib::QNX_MAX_PRIORITY - 3);

	// Attach signal handlers.
	signal(SIGTERM, &vsp::int_shell::catch_signal);
	signal(SIGSEGV, &vsp::int_shell::catch_signal);
#if defined(PROCESS_SPAWN_RSH)
	signal(SIGINT, SIG_IGN);
#endif

	// Check number of arguments.
	if (argc <= 6) {
		printf("Number of required arguments is insufficient.\n");
		return -1;
	}

	// Read system configuration.
	lib::configurator _config(argv[1], argv[2], argv[3], argv[4], argv[5]);

	resourceman_attach_point = _config.return_attach_point_name(lib::configurator::CONFIG_RESOURCEMAN_LOCAL, "resourceman_attach_point");

	try {
		// Create sensor - abstract factory pattern.
		vsp::int_shell::vs = vsp::common::return_created_sensor(_config);

		// Check resource attach point.
		if (access(resourceman_attach_point.c_str(), R_OK) == 0) {
			throw vsp::common::vsp_error(lib::SYSTEM_ERROR, DEVICE_EXISTS);
		}

		// Initialize dispatch interface.
		if ((dpp = dispatch_create()) == NULL)
			throw vsp::common::vsp_error(lib::SYSTEM_ERROR, DISPATCH_ALLOCATION_ERROR);

		// Iinitialize resource manager attributes.
		memset(&resmgr_attr, 0, sizeof resmgr_attr);
		resmgr_attr.nparts_max = 1;
		resmgr_attr.msg_max_size = sizeof(mrrocpp::vsp::common::DEVCTL_MSG);

		// Iinitialize functions for handling messages.
		iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS, &io_funcs);
		io_funcs.read = vsp::int_shell::io_read;
		io_funcs.write = vsp::int_shell::io_write;
		io_funcs.devctl = vsp::int_shell::io_devctl;

		// Initialize attribute structure used by the device.
		iofunc_attr_init(&attr, S_IFNAM | 0666, 0, 0);
		attr.nbytes = sizeof(mrrocpp::vsp::common::DEVCTL_MSG);

		// Attach device name.
		if ((id = resmgr_attach(dpp, /* dispatch handle        */
		&resmgr_attr, /* resource manager attrs */
		resourceman_attach_point.c_str(), /* device name            */
		_FTYPE_ANY, /* open type              */
		0, /* flags                  */
		&connect_funcs, /* connect routines       */
		&io_funcs, /* I/O routines           */
		&attr)) == -1) { /* handle                 */
			throw vsp::common::vsp_error(lib::SYSTEM_ERROR, DEVICE_CREATION_ERROR);
		}

		// Allocate a context structure.
		ctp = dispatch_context_alloc(dpp);

		// Start the resource manager message loop.
		vsp::int_shell::vs->sr_msg->message("Device is waiting for clients...");
		while (!vsp::int_shell::TERMINATED) { // for(;;)
			if ((ctp = dispatch_block(ctp)) == NULL)
				throw vsp::common::vsp_error(lib::SYSTEM_ERROR, DISPATCH_LOOP_ERROR);
			dispatch_handler(ctp);
		} //: for
		vsp::int_shell::vs->sr_msg->message("vsp  terminated");
		delete mrrocpp::vsp::int_shell::vs;
	} //: try
	catch (vsp::common::vsp_error & e) {
		vsp::int_shell::error_handler(e);
		exit(EXIT_FAILURE);
	} //: catch
} //: main

