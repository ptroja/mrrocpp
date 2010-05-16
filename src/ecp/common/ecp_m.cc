// ------------------------------------------------------------------------
//   ecp_m.cc - szablon dla procesow ECP
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "ecp/common/task/ecp_task.h"
#include "ecp/common/ECP_main_error.h"
#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

common::task::task *ecp_t;

void catch_signal_in_ecp(int sig)
{
	fprintf(stderr, "ECP: %s\n", strsignal(sig));
	switch (sig) {
		// print info message
		case SIGTERM:
			ecp_t->sh_msg->message("ECP terminated");
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
		if(argc < 6) {
			printf("Za malo argumentow ECP\n");
			return -1;
		}

		// TODO: this should not be a pointer; blcked by error handling fixup
		// configuration read
		lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3], argv[4], argv[5]);

		ecp::common::ecp_t = ecp::common::task::return_created_ecp_task(*_config);

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY-3);

		signal(SIGTERM, &(ecp::common::catch_signal_in_ecp));
		signal(SIGSEGV, &(ecp::common::catch_signal_in_ecp));
#if defined(PROCESS_SPAWN_RSH)
		// ignore Ctrl-C signal, which cames from UI console
		signal(SIGINT, SIG_IGN);
#endif
	}
	catch (ecp_mp::task::ECP_MP_main_error & e) {
		if (e.error_class == lib::SYSTEM_ERROR)
			exit(EXIT_FAILURE);
	}
	catch (ecp::common::ecp_robot::ECP_main_error & e) {
		switch (e.error_class ) {
			case lib::SYSTEM_ERROR:
			case lib::FATAL_ERROR:
				ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, e.error_no);
				exit(EXIT_FAILURE);
				break;
			default:
				break;
		}
	}
	catch (ecp::common::generator::generator::ECP_error & e) {
		ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, e.error_no);
		printf("Mam blad generatora section 1 (@%s:%d)\n", __FILE__, __LINE__);
	}
	catch (lib::sensor::sensor_error & e) {
		ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, e.error_no);
		printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
	}
	catch (ecp_mp::transmitter::transmitter_base::transmitter_error & e) {
		ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, 0);
		printf("ecp_m.cc: Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);
	}

	catch(const std::exception& e){
		std::string tmp_string(" The following error has been detected: ");
		tmp_string += e.what();
		ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, tmp_string.c_str());
	   std::cerr<<"ECP: The following error has been detected :\n\t"<<e.what()<<std::endl;
	}

	catch (...) {  /* Dla zewnetrznej petli try*/
		/* Wylapywanie niezdefiniowanych bledow*/
		/*Komunikat o bledzie wysylamy do SR*/
		ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, ECP_UNIDENTIFIED_ERROR);
		exit(EXIT_FAILURE);
	} /*end: catch */

	for (;;) { // Zewnetrzna petla nieskonczona

		try {
			ecp::common::ecp_t->sr_ecp_msg->message("Press START");
			ecp::common::ecp_t->ecp_wait_for_start();

			ecp::common::ecp_t->main_task_algorithm();

			ecp::common::ecp_t->ecp_wait_for_stop();
			ecp::common::ecp_t->sr_ecp_msg->message("Press STOP");
		}

		catch (ecp_mp::task::ECP_MP_main_error & e) {
			if (e.error_class == lib::SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		}
		catch (ecp::common::ECP_main_error & e) {
			if (e.error_class == lib::SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		}

		catch (ecp::common::ecp_robot::ECP_error & er) {
			/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP*/
			if ( er.error_class == lib::SYSTEM_ERROR) { /*blad systemowy juz wyslano komunukat do SR*/
				perror("ECP aborted due to lib::SYSTEM_ERRORn");
				exit(EXIT_FAILURE);
			}

			switch ( er.error_no ) {
				case INVALID_POSE_SPECIFICATION:
				case INVALID_ECP_COMMAND:
				case INVALID_COMMAND_TO_EDP:
				case EDP_ERROR:
				case INVALID_EDP_REPLY:
				case INVALID_ROBOT_MODEL_TYPE:
					/*Komunikat o bledzie wysylamy do SR */
					ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, er.error_no);
					ecp::common::ecp_t->set_ecp_reply (lib::ERROR_IN_ECP);
					ecp::common::ecp_t->mp_buffer_receive_and_send();
					break;
				default:
					ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, 0, "ECP: Unidentified exception");
					perror("Unidentified exception");
					exit(EXIT_FAILURE);
			} /* end: switch */
		} /*end: catch*/

		catch (ecp::common::generator::generator::ECP_error & er) {
			/* Wylapywanie bledow generowanych przez generatory*/
			if ( er.error_class == lib::SYSTEM_ERROR) { /* blad systemowy juz wyslano komunukat do SR */
				perror("ECP aborted due to lib::SYSTEM_ERROR");
				exit(EXIT_FAILURE);
			}
			switch ( er.error_no ) {
				case INVALID_POSE_SPECIFICATION:
				case INVALID_MP_COMMAND:
				case NON_EXISTENT_DIRECTORY:
				case NON_TRAJECTORY_FILE:
				case NON_EXISTENT_FILE:
				case READ_FILE_ERROR:
				case NON_COMPATIBLE_LISTS:
				case MAX_ACCELERATION_EXCEEDED:
				case MAX_VELOCITY_EXCEEDED:
				case NOT_ENOUGH_MEMORY:
					/*Komunikat o bledzie wysylamy do SR */
					ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, er.error_no);
					ecp::common::ecp_t->set_ecp_reply (lib::ERROR_IN_ECP);
					ecp::common::ecp_t->mp_buffer_receive_and_send();
					break;
				case ECP_STOP_ACCEPTED:
					ecp::common::ecp_t->sr_ecp_msg->message("pierwszy catch stop");
					break;
				default:
					ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, 0, "ECP: Unidentified exception");
					perror("Unidentified exception");
					exit(EXIT_FAILURE);
			} /* end: switch*/
		} /*end: catch */
		catch (lib::sensor::sensor_error & e) {
			ecp::common::ecp_t->sr_ecp_msg->message (e.error_class, e.error_no);
			printf("Mam blad czujnika section 2 (@%s:%d)\n", __FILE__, __LINE__);
		}
		catch (ecp_mp::transmitter::transmitter_base::transmitter_error & e) {
			ecp::common::ecp_t->sr_ecp_msg->message (e.error_class, 0);
			printf("Mam blad trasnmittera section 2 (@%s:%d)\n", __FILE__, __LINE__);
		}

		catch(const std::exception& e){
			std::string tmp_string(" The following error has been detected: ");
			tmp_string += e.what();
			ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, tmp_string.c_str());
		   std::cerr<<"ECP: The following error has been detected :\n\t"<<e.what()<<std::endl;
		}

		catch (...) {  /* Dla zewnetrznej petli try*/
			/* Wylapywanie niezdefiniowanych bledow*/
			/*Komunikat o bledzie wysylamy do SR*/
			ecp::common::ecp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, ECP_UNIDENTIFIED_ERROR);
			exit(EXIT_FAILURE);
		} /*end: catch */

		ecp::common::ecp_t->sr_ecp_msg->message("ECP user program is finished");

	} // end: for (;;) zewnetrznej

} // koniec: main()
// ------------------------------------------------------------------------
