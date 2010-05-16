// ------------------------------------------------------------------------
//
//                      MASTER PROCESS (MP) - main()
//
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "mp/mp.h"


namespace mrrocpp {
namespace mp {
namespace common {

// obiekt z metodami i polami dla procesu MP (polimorficzny)
task::task* mp_t;

void catch_signal_in_mp(int sig)
{
	// print info message
	fprintf(stderr, "MP: %s\n", strsignal(sig));
	pid_t child_pid;
	int status;
	switch (sig) {
	case SIGTERM:
		mp_t->sh_msg->message("MP terminated");
		// restore default (none) handler for SIGCHLD
		signal(SIGCHLD, SIG_DFL);
		delete mp_t;
		exit(EXIT_SUCCESS);
		break;
	case SIGSEGV:
		signal(SIGSEGV, SIG_DFL);
		break;
	case SIGCHLD:
		child_pid = waitpid(-1, &status, 0);
		if (child_pid == -1) {
			perror("MP: waitpid()");
		} else if (child_pid == 0) {
			fprintf(stderr, "MP: no child exited\n");
		} else {
			//fprintf(stderr, "UI: child %d...\n", child_pid);
			if (WIFEXITED(status)) {
				fprintf(stderr, "MP: child %d exited normally with status %d\n",
						child_pid, WEXITSTATUS(status));
			}
			if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
				if (WCOREDUMP(status)) {
					fprintf(stderr, "MP: child %d terminated by signal %d (core dumped)\n",
							child_pid, WTERMSIG(status));
				}
				else
#endif /* WCOREDUMP */
				{
					fprintf(stderr, "MP: child %d terminated by signal %d\n",
							child_pid, WTERMSIG(status));
				}
			}
			if (WIFSTOPPED(status)) {
				fprintf(stderr, "MP: child %d stopped\n", child_pid);
			}
			if (WIFCONTINUED(status)) {
				fprintf(stderr, "MP: child %d resumed\n", child_pid);
			}
		}
		break;
	}
	flushall();
}

} // namespace common
} // namespace mp
} // namespace mrrocpp



int main (int argc, char *argv[], char **arge)
{
	// zewnetrzne try
	try {

		if (argc < 6) {
			printf("Usage: mp_m_c <ui_node_name> <mrrocpp_local_path> <config_file> <session_name>\n");
			exit(EXIT_FAILURE);
		}

		try	{
			// TODO: new/delete fixup
			lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3], MP_SECTION, argv[5]);

			mp::common::mp_t = mp::task::return_created_mp_task(*_config);

			mp::common::mp_t->sr_ecp_msg->message("MP loaded");

			lib::set_thread_priority(pthread_self(), MAX_PRIORITY-4);

			signal(SIGTERM, &(mp::common::catch_signal_in_mp));
			//signal(SIGINT,  &(catch_signal_in_mp));
			signal(SIGSEGV, &(mp::common::catch_signal_in_mp));
			signal(SIGCHLD, &(mp::common::catch_signal_in_mp));
#if defined(PROCESS_SPAWN_RSH)
			// ignore Ctrl-C signal, which cames from UI console
			signal(SIGINT, SIG_IGN);
#endif
		}
		catch (ecp_mp::task::ECP_MP_main_error & e) {
			/* Obsluga bledow ECP_MP_main_error */
			if (e.error_class == lib::SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		} /*end: catch */
		catch (mp::common::MP_main_error & e) {

			perror("initialize incorrect");
			if (e.error_class == lib::SYSTEM_ERROR)
				exit(EXIT_FAILURE);

			/* Obsluga lib::NON_FATAL_ERROR*/
			switch (e.error_no) {
				case ECP_ERRORS:
				case INVALID_ECP_PULSE_IN_MP_START_ALL:
				case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
				case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
					mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					break;
				default:
					perror("Message to SR has been already sent");
			}/* end:switch */
			mp::common::mp_t->sr_ecp_msg->message("To terminate MP click STOP icon");
		} /*end: catch*/

		catch (lib::sensor::sensor_error & e) {
			/* Wyswietlenie komunikatu. */
			mp::common::mp_t->sr_ecp_msg->message (e.error_class, e.error_no);
			printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */
		catch (mp::generator::generator::MP_error & e) {
					/* Wyswietlenie komunikatu. */
			mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
					printf("Mam blad mp_generator section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */
		catch (ecp_mp::transmitter::transmitter_base::transmitter_error & e) {
			/* Wyswietlenie komunikatu. */
			mp::common::mp_t->sr_ecp_msg->message (e.error_class, 0);
			printf("Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */

		catch(const std::exception& e){
			std::string tmp_string(" The following error has been detected: ");
			tmp_string += e.what();
			mp::common::mp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, tmp_string.c_str());
		   std::cerr<<"MP: The following error has been detected :\n\t"<<e.what()<<std::endl;
		}


		catch (...) {  /* Dla zewnetrznej petli try*/
			/*   Wylapywanie niezdfiniowanych bledow  */
			/*  Komunikat o bledzie wysylamy do SR */
			mp::common::mp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
			exit(EXIT_FAILURE);
		} /*end: catch  */

		for (;;) {  // Wewnetrzna petla nieskonczona

			try {
				mp::common::mp_t->sr_ecp_msg->message("MP - wcisnij start");

				// Oczekiwanie na zlecenie START od UI
				mp::common::mp_t->wait_for_start();

				// Wyslanie START do wszystkich ECP
				mp::common::mp_t->start_all(mp::common::mp_t->robot_m);

				mp::common::mp_t->main_task_algorithm();

				// Oczekiwanie na STOP od UI
				mp::common::mp_t->wait_for_stop();

				// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
				mp::common::mp_t->terminate_all (mp::common::mp_t->robot_m);
			}  // end: try

			catch (ecp_mp::task::ECP_MP_main_error & e) {
				/* Obsluga bledow ECP_MP_main_error */
				if (e.error_class == lib::SYSTEM_ERROR)
					exit(EXIT_FAILURE);
			} /*end: catch */
			catch (mp::common::MP_main_error & e) {

				if (e.error_class == lib::SYSTEM_ERROR)
					exit(EXIT_FAILURE);

				/* Obsluga lib::NON_FATAL_ERROR */
				switch (e.error_no) {
					case ECP_ERRORS:
					case INVALID_ECP_PULSE_IN_MP_START_ALL:
					case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
					case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
						mp::common::mp_t->stop_and_terminate();
						break;
					case ECP_STOP_ACCEPTED:
						mp::common::mp_t->sr_ecp_msg->message("ECP STOP ACCEPTED");
					break;
					default:
						perror("Unidentified mp error");
						mp::common::mp_t->stop_and_terminate();
				}/*end:switch*/



			} /*end: catch */
			catch (mp::robot::robot::MP_error & e) {
				if (e.error_class == lib::SYSTEM_ERROR) {
					exit(EXIT_FAILURE);
				}

				/* Obsluga lib::NON_FATAL_ERROR */
				switch (e.error_no) {
					case ECP_ERRORS:
					case INVALID_POSE_SPECIFICATION:
					case INVALID_ECP_COMMAND:
					case INVALID_COMMAND_TO_EDP:
					case EDP_ERROR:
					case INVALID_EDP_REPLY:
					case INVALID_ROBOT_MODEL_TYPE:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
						break;
					default:
						perror("Unidentified mp error");
				}/*end:switch */
				mp::common::mp_t->stop_and_terminate();

			} /*end: catch*/

			catch (mp::generator::generator::MP_error & e) {

				if (e.error_class == lib::SYSTEM_ERROR)
					exit(EXIT_FAILURE);

				/* Obsluga lib::NON_FATAL_ERROR*/
				switch (e.error_no) {
					case ECP_ERRORS:
					case INVALID_POSE_SPECIFICATION:
					case NON_EXISTENT_DIRECTORY:
					case NON_TRAJECTORY_FILE:
					case NON_EXISTENT_FILE:
					case READ_FILE_ERROR:
					case NON_COMPATIBLE_LISTS:
					case MAX_ACCELERATION_EXCEEDED:
					case MAX_VELOCITY_EXCEEDED:
					case NOT_ENOUGH_MEMORY:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
						break;
					default:
						perror("Unidentified mp error");
				}/* end:switch*/
				mp::common::mp_t->stop_and_terminate();

			} /*end: catch*/

			catch (lib::sensor::sensor_error & e) {
				/* Wyswietlenie komunikatu. */
				mp::common::mp_t->sr_ecp_msg->message (e.error_class, e.error_no);
				printf("Mam blad czujnika section 2 (@%s:%d)\n", __FILE__, __LINE__);
			} /* end: catch sensor_error  */
			catch (ecp_mp::transmitter::transmitter_base::transmitter_error & e) {
				/* Wyswietlenie komunikatu. */
				mp::common::mp_t->sr_ecp_msg->message (e.error_class, 0);
				printf("Mam blad trasnmittera section 2 (@%s:%d)\n", __FILE__, __LINE__);
			} /* end: catch sensor_error  */

			catch(const std::exception& e){
				std::string tmp_string(" The following error has been detected: ");
				tmp_string += e.what();
				mp::common::mp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, tmp_string.c_str());
			   std::cerr<<"MP: The following error has been detected :\n\t"<<e.what()<<std::endl;
			}

			catch (...) {  /* Dla zewnetrznej petli try*/
				/*   Wylapywanie niezdfiniowanych bledow  */
				/*  Komunikat o bledzie wysylamy do SR */
				mp::common::mp_t->sr_ecp_msg->message (lib::NON_FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
				exit(EXIT_FAILURE);
			} /*end: catch  */

		} // koniec: for(;;) - zewnetrzna petla

	}
	catch (...) {  /* Dla zewnetrznej petli try*/
		/* Wylapywanie niezdefiniowanych bledow  */
		/* Komunikat o bledzie wysylamy do SR */
		printf("unexpected exception throw from catch section (@%s:%d)\n", __FILE__, __LINE__);
		mp::common::mp_t->sr_ecp_msg->message (lib::FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
		exit(EXIT_FAILURE);
	} /* end: catch  */

}
