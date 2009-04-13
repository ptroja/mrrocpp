// ------------------------------------------------------------------------
//   ecp_m.cc - szablon dla procesow ECP
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "ecp/common/ecp_task.h"
#include "ecp/common/ECP_main_error.h"
#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

common::task::base *ecp_t;

void catch_signal_in_ecp(int sig)
{
	ecp_t->catch_signal_in_ecp_task(sig);
	delete ecp_t;
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

		// configuration read
		lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3], argv[4], argv[5]);
#if defined(PROCESS_SPAWN_YRSH)
		if (argc>6) {
			_config->answer_to_y_rsh_spawn(argv[6]);
			signal(SIGINT, SIG_IGN);
		}
#endif
		ecp::common::ecp_t = ecp::common::task::return_created_ecp_task(*_config);

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY-3);

		signal(SIGTERM, &(ecp::common::catch_signal_in_ecp));
		signal(SIGSEGV, &(ecp::common::catch_signal_in_ecp));
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif

		ecp::common::ecp_t->initialize_communication();

		ecp::common::ecp_t->task_initialization();
	}
	catch (ecp_mp::task::ECP_MP_main_error e) {
		if (e.error_class == SYSTEM_ERROR)
			exit(EXIT_FAILURE);
	}
	catch (ecp::common::ecp_robot::ECP_main_error e) {
		if (e.error_class == SYSTEM_ERROR)
			exit(EXIT_FAILURE);
	}
	catch (ecp::common::generator::base::ECP_error e) {
		ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, e.error_no);
		printf("Mam blad generatora section 1 (@%s:%d)\n", __FILE__, __LINE__);
	}
	catch (::sensor::sensor_error e) {
		ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, e.error_no);
		printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
	}
	catch (ecp_mp::transmitter::base::transmitter_error e) {
		ecp::common::ecp_t->sr_ecp_msg->message(e.error_class, e.error_no);
		printf("ecp_m.cc: Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);
	}

	catch (...) {  /* Dla zewnetrznej petli try*/
		/* Wylapywanie niezdefiniowanych bledow*/
		/*Komunikat o bledzie wysylamy do SR*/
		ecp::common::ecp_t->sr_ecp_msg->message (NON_FATAL_ERROR, (uint64_t) ECP_UNIDENTIFIED_ERROR);
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

		catch (ecp_mp::task::ECP_MP_main_error e) {
			if (e.error_class == SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		}
		catch (ecp::common::ECP_main_error e) {
			if (e.error_class == SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		}

		catch (ecp::common::ecp_robot::ECP_error er) {
			/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP*/
			if ( er.error_class == SYSTEM_ERROR) { /*blad systemowy juz wyslano komunukat do SR*/
				perror("ECP aborted due to SYSTEM_ERRORn");
				exit(EXIT_FAILURE);
			}

			switch ( er.error_no ) {
				case INVALID_POSE_SPECIFICATION:
				case INVALID_ECP_COMMAND:
				case INVALID_COMMAND_TO_EDP:
				case EDP_ERROR:
				case INVALID_EDP_REPLY:
				case INVALID_RMODEL_TYPE:
					/*Komunikat o bledzie wysylamy do SR */
					ecp::common::ecp_t->sr_ecp_msg->message (NON_FATAL_ERROR, er.error_no);
					ecp::common::ecp_t->set_ecp_reply (ERROR_IN_ECP);
					ecp::common::ecp_t->mp_buffer_receive_and_send();
					break;
				default:
					ecp::common::ecp_t->sr_ecp_msg->message (NON_FATAL_ERROR, 0, "ECP: Unidentified exception");
					perror("Unidentified exception");
					exit(EXIT_FAILURE);
			} /* end: switch */
		} /*end: catch*/

		catch (ecp::common::generator::base::ECP_error er) {
			/* Wylapywanie bledow generowanych przez generatory*/
			if ( er.error_class == SYSTEM_ERROR) { /* blad systemowy juz wyslano komunukat do SR */
				perror("ECP aborted due to SYSTEM_ERROR");
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
					ecp::common::ecp_t->sr_ecp_msg->message (NON_FATAL_ERROR, er.error_no);
					ecp::common::ecp_t->set_ecp_reply (ERROR_IN_ECP);
					ecp::common::ecp_t->mp_buffer_receive_and_send();
					break;
				case ECP_STOP_ACCEPTED:
					ecp::common::ecp_t->sr_ecp_msg->message("pierwszy catch stop");
					break;
				default:
					ecp::common::ecp_t->sr_ecp_msg->message (NON_FATAL_ERROR, 0, "ECP: Unidentified exception");
					perror("Unidentified exception");
					exit(EXIT_FAILURE);
			} /* end: switch*/
		} /*end: catch */
		catch (sensor::sensor_error e) {
			ecp::common::ecp_t->sr_ecp_msg->message (e.error_class, e.error_no);
			printf("Mam blad czujnika section 2 (@%s:%d)\n", __FILE__, __LINE__);
		}
		catch (ecp_mp::transmitter::base::transmitter_error e) {
			ecp::common::ecp_t->sr_ecp_msg->message (e.error_class, e.error_no);
			printf("Mam blad trasnmittera section 2 (@%s:%d)\n", __FILE__, __LINE__);
		}

		catch (...) {  /* Dla zewnetrznej petli try*/
			/* Wylapywanie niezdefiniowanych bledow*/
			/*Komunikat o bledzie wysylamy do SR*/
			ecp::common::ecp_t->sr_ecp_msg->message (NON_FATAL_ERROR, (uint64_t) ECP_UNIDENTIFIED_ERROR);
			exit(EXIT_FAILURE);
		} /*end: catch */

		ecp::common::ecp_t->sr_ecp_msg->message("ECP user program is finished");

	} // end: for (;;) zewnetrznej

} // koniec: main()
// ------------------------------------------------------------------------
