/*!
 \file vsp_m_int_srr.cc

 \brief file contains the interactive VSP shell

 The interactive VSP shell should be used for all sensors,  that are required to cooperate with MP/ECP processes interativelly, which means that they should perform reading initiation and aggregation to the form useful in control only when such command will be received.


 \date 11.10.2007
 \author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 \author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology

 \defgroup VSP -- Virtual Sensor Process
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/configurator.h"
#include "base/vsp/vsp_sensor.h"
#include "base/vsp/vsp_error.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

namespace mrrocpp {
namespace vsp {
namespace common {

/** Global pointer to sensor object. */
static sensor::sensor_interface *vs;

/** Threads termination flag. */
static bool TERMINATE = false;


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
			printf("vsp  aborted due to lib::SYSTEM_ERROR\n");
			vs->sr_msg->message(lib::SYSTEM_ERROR, e.error_no);
			TERMINATE = true;
			break;
		case lib::FATAL_ERROR:
			vs->sr_msg->message(lib::FATAL_ERROR, e.error_no);
			break;
		case lib::NON_FATAL_ERROR:
			switch (e.error_no)
			{
				case INVALID_COMMAND_TO_VSP:
					vs->from_vsp.vsp_report = lib::INVALID_VSP_COMMAND;
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				case SENSOR_NOT_CONFIGURED:
					vs->from_vsp.vsp_report = lib::VSP_SENSOR_NOT_CONFIGURED;
					vs->sr_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				case READING_NOT_READY:
					vs->from_vsp.vsp_report = lib::VSP_READING_NOT_READY;
					break;
				default:
					vs->sr_msg->message(lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
			}
			break;
		default:
			vs->sr_msg->message(lib::NON_FATAL_ERROR, VSP_UNIDENTIFIED_ERROR);
	} //: switch
} //: error_handler

} // namespace common
} // namespace vsp
} // namespace mrrocpp

/**
 * @brief Main body of the interactive VSP shell.
 * @param argc Number of passed arguments
 * @param argv Process arguments - first five are parameters of the Configurator object
 * @return Status
 */
int main(int argc, char *argv[])
{

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

	const std::string attach_point =
			_config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point");

	try {

		// Create sensor - abstract factory sensor.
		vsp::common::vs = vsp::sensor::return_created_sensor(_config);

		// Create messip communication channel.
		messip_channel_t *ch;
		if ((ch = messip::port_create(attach_point)) == NULL) {
			fprintf(stderr, "creating channel failed\n");
			return -1;
		}

		// Start the resource manager message loop.
		vsp::common::vs->sr_msg->message("Device is waiting for clients...");

		// Main processing loop.
		while (!vsp::common::TERMINATE) {
			int32_t type, subtype;

			// Receive message via messip port..
			int rcvid = messip::port_receive(ch, type, subtype, vsp::common::vs->to_vsp);

			 // Check error condition.
			if (rcvid == -1)
			{
				perror("vsp: Receive failed");
				break;
			} else if (rcvid < -1) {
				// ie. MESSIP_MSG_DISCONNECT
				fprintf(stderr, "vsp: ie. MESSIP_MSG_DISCONNECT\n");
				continue;
			}

			 // Set default report.
			vsp::common::vs->from_vsp.vsp_report = lib::VSP_REPLY_OK;

			// Check command.
			try {
				switch (vsp::common::vs->to_vsp.i_code)
				{
					case lib::VSP_CONFIGURE_SENSOR:
						vsp::common::vs->configure_sensor();
						break;
					case lib::VSP_INITIATE_READING:
						vsp::common::vs->initiate_reading();
						break;
					case lib::VSP_GET_READING:
						vsp::common::vs->get_reading();
						break;
					case lib::VSP_TERMINATE:
						delete vsp::common::vs;
						vsp::common::TERMINATE = true;
						break;
					default:
						throw lib::vsp_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_VSP);
				}
			}

			catch (lib::vsp_error & e) {
				vsp::common::error_handler(e);
			} //: catch
			catch (lib::sensor::sensor_error & e) {
				vsp::common::error_handler(e);
			} //: catch

			// Send message via reply port.
			messip::port_reply(ch, rcvid, 0, vsp::common::vs->from_vsp);
		} //: while
		vsp::common::vs->sr_msg->message("vsp  terminated");
	} //: try
	catch (lib::vsp_error & e) {
		vsp::common::error_handler(e);
		exit(EXIT_FAILURE);
	} //: catch
} //: main
