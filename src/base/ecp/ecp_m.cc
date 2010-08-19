// ------------------------------------------------------------------------
//   ecp_m.cc - szablon dla procesow ECP
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <cstdio>
#include <csignal>
#include <cstdlib>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_generator.h"

#include "lib/exception.h"
#include <boost/exception/diagnostic_information.hpp>

namespace mrrocpp {
namespace ecp {
namespace common {

common::task::task *ecp_t;

void catch_signal_in_ecp(int sig)
{
	fprintf(stderr, "ecp: %s\n", strsignal(sig));
	switch (sig)
	{
		// print info message
		case SIGTERM:
			ecp_t->sh_msg->message("ecp terminated");
			delete ecp_t;
			exit(EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in ECP process %s\n", ecp_t->config.section_name.c_str());
			signal(SIGSEGV, SIG_DFL);
			break;
	}
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp

int main(int argc, char *argv[])
{
	try {

		// liczba argumentow
		if (argc < 6) {
			printf("Za malo argumentow ECP\n");
			return -1;
		}

		// TODO: this should not be a pointer; blcked by error handling fixup
		// configuration read
		lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3], argv[4], argv[5]);

		ecp::common::ecp_t = ecp::common::task::return_created_ecp_task(*_config);

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 3);

		signal(SIGTERM, &(ecp::common::catch_signal_in_ecp));
		signal(SIGSEGV, &(ecp::common::catch_signal_in_ecp));
#if defined(PROCESS_SPAWN_RSH)
		// ignore Ctrl-C signal, which cames from UI console
		signal(SIGINT, SIG_IGN);
#endif

		for (;;) { // Zewnetrzna petla nieskonczona

			try {
				ecp::common::ecp_t->sr_ecp_msg->message("Press START");
				ecp::common::ecp_t->wait_for_start();

				ecp::common::ecp_t->main_task_algorithm();

				ecp::common::ecp_t->wait_for_stop();
				ecp::common::ecp_t->sr_ecp_msg->message("Press STOP");
			}

	/*
			case INVALID_POSE_SPECIFICATION:
			case INVALID_COMMAND_TO_EDP:
			case EDP_ERROR:
			case INVALID_ROBOT_MODEL_TYPE:
				ecp::common::ecp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, er.error_no);
				ecp::common::ecp_t->set_ecp_reply(lib::ERROR_IN_ECP);
				ecp::common::ecp_t->mp_buffer_receive_and_send();
	*/

	/*
			case INVALID_POSE_SPECIFICATION:
			case INVALID_MP_COMMAND:
			case NON_EXISTENT_DIRECTORY:
			case NON_TRAJECTORY_FILE:
			case NON_EXISTENT_FILE:
			case READ_FILE_ERROR:
			case NON_COMPATIBLE_LISTS:
			case MAX_ACCELERATION_EXCEEDED:
			case MAX_VELOCITY_EXCEEDED:
				Komunikat o bledzie wysylamy do SR
				ecp::common::ecp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, er.error_no);
				ecp::common::ecp_t->set_ecp_reply(lib::ERROR_IN_ECP);
				ecp::common::ecp_t->mp_buffer_receive_and_send();
				break;
			case ECP_STOP_ACCEPTED:
				ecp::common::ecp_t->sr_ecp_msg->message("pierwszy catch stop");
				break;
	*/
			catch (lib::exception::NonFatal_error & e) {
				std::cerr << "Exception in ECP:" << std::endl;
				std::cerr << diagnostic_information(e);
				ecp::common::ecp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, ECP_UNIDENTIFIED_ERROR);
			}

			ecp::common::ecp_t->sr_ecp_msg->message("ECP user program is finished");
		}

	}

	catch (const boost::exception & e ) {
		std::cerr << "Exception in ECP:" << std::endl;
		std::cerr << diagnostic_information(e);
		const std::string message(diagnostic_information(e));
		ecp::common::ecp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, message);
	}

	catch (const std::exception & e) {
		std::cerr << "Exception in ECP:" << std::endl;
		std::string message("The following error has been detected: ");
		message += e.what();
		ecp::common::ecp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, message);
		std::cerr << "ECP: The following error has been detected :\n\t" << e.what() << std::endl;
	}

	catch (...) {
		/* Wylapywanie niezdefiniowanych bledow*/
		/*Komunikat o bledzie wysylamy do SR*/
		ecp::common::ecp_t->sr_ecp_msg->message(lib::FATAL_ERROR, ECP_UNIDENTIFIED_ERROR);
		exit(EXIT_FAILURE);
	}

	return 0;
}
