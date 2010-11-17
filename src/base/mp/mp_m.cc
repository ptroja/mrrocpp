/*!
 * @file
 * @brief File contains main mp loop definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>

#include "base/ecp_mp/transmitter.h"

#include "base/lib/mis_fun.h"

#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_generator.h"
#include "base/mp/mp_robot.h"
#include "base/mp/MP_main_error.h"

namespace mrrocpp {
namespace mp {
namespace common {

// obiekt z metodami i polami dla procesu MP (polimorficzny)
task::task* mp_t;

void catch_signal_in_mp(int sig)
{
	// print info message
	fprintf(stderr, "mp: %s\n", strsignal(sig));
	pid_t child_pid;
	int status;
	switch (sig)
	{
		case SIGTERM:
			mp_t->sh_msg->message("mp terminated");
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
				perror("mp: waitpid()");
			} else if (child_pid == 0) {
				fprintf(stderr, "mp: no child exited\n");
			} else {
				//fprintf(stderr, "UI: child %d...\n", child_pid);
				if (WIFEXITED(status)) {
					fprintf(stderr, "mp: child %d exited normally with status %d\n", child_pid, WEXITSTATUS(status));
				}
				if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
					if (WCOREDUMP(status)) {
						fprintf(stderr, "mp: child %d terminated by signal %d (core dumped)\n", child_pid, WTERMSIG(status));
					} else
#endif /* WCOREDUMP */
					{
						fprintf(stderr, "mp: child %d terminated by signal %d\n", child_pid, WTERMSIG(status));
					}
				}
				if (WIFSTOPPED(status)) {
					fprintf(stderr, "mp: child %d stopped\n", child_pid);
				}
				if (WIFCONTINUED(status)) {
					fprintf(stderr, "mp: child %d resumed\n", child_pid);
				}
			}
			break;
	}
	flushall();
}

} // namespace common
} // namespace mp
} // namespace mrrocpp


int main(int argc, char *argv[], char **arge)
{
	// zewnetrzne try
	try {
		std::cerr << "mp 1" << std::endl;

		if (argc < 6) {
			printf("Usage: mp_m_c <ui_node_name> <mrrocpp_local_path> <config_file> <session_name>\n");
			exit(EXIT_FAILURE);
		}

		try {
			// TODO: new/delete fixup
			std::cerr << "mp 2" << std::endl;
			lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3], lib::MP_SECTION, argv[5]);
			std::cerr << "mp 3" << std::endl;
			mp::common::mp_t = mp::task::return_created_mp_task(*_config);
			std::cerr << "mp 4" << std::endl;
			// Utworzenie listy robotow, powolanie procesow ECP i nawiazanie komunikacji z nimi
			mp::common::mp_t->create_robots();
			std::cerr << "mp 5" << std::endl;

			mp::common::mp_t->sr_ecp_msg->message("mp loaded");
			std::cerr << "mp 6" << std::endl;

			lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 4);

			signal(SIGTERM, &(mp::common::catch_signal_in_mp));
			//signal(SIGINT,  &(catch_signal_in_mp));
			signal(SIGSEGV, &(mp::common::catch_signal_in_mp));
			signal(SIGCHLD, &(mp::common::catch_signal_in_mp));
#if defined(PROCESS_SPAWN_RSH)
			// ignore Ctrl-C signal, which cames from UI console
			signal(SIGINT, SIG_IGN);
#endif
		} catch (ecp_mp::task::ECP_MP_main_error & e) {
			/* Obsluga bledow ECP_MP_main_error */
			if (e.error_class == lib::SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		} /*end: catch */
		catch (mp::common::MP_main_error & e) {

			perror("initialize incorrect");
			if (e.error_class == lib::SYSTEM_ERROR)
				exit(EXIT_FAILURE);

			/* Obsluga lib::NON_FATAL_ERROR*/
			switch (e.error_no)
			{
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
			mp::common::mp_t->sr_ecp_msg->message(e.error_class, e.error_no);
			printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */
		catch (mp::generator::MP_error & e) {
			/* Wyswietlenie komunikatu. */
			mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
			printf("Mam blad mp_generator section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */
		catch (ecp_mp::transmitter::transmitter_error & e) {
			/* Wyswietlenie komunikatu. */
			mp::common::mp_t->sr_ecp_msg->message(e.error_class, 0);
			printf("Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */

		catch (const std::exception& e) {
			std::string tmp_string(" The following error has been detected: ");
			tmp_string += e.what();
			mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, tmp_string.c_str());
			std::cerr << "mp: The following error has been detected :\n\t" << e.what() << std::endl;
		}

		catch (...) { /* Dla zewnetrznej petli try*/
			/*   Wylapywanie niezdfiniowanych bledow  */
			/*  Komunikat o bledzie wysylamy do SR */
			mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
			exit(EXIT_FAILURE);
		} /*end: catch  */

		for (;;) { // Wewnetrzna petla nieskonczona

			try {
				mp::common::mp_t->sr_ecp_msg->message("mp - wcisnij start");

				// Oczekiwanie na zlecenie START od UI
				mp::common::mp_t->wait_for_start();

				// Wyslanie START do wszystkich ECP
				mp::common::mp_t->start_all(mp::common::mp_t->robot_m);

				mp::common::mp_t->main_task_algorithm();

				// Oczekiwanie na STOP od UI
				mp::common::mp_t->wait_for_stop();

				// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
				mp::common::mp_t->terminate_all(mp::common::mp_t->robot_m);
			} // end: try

			catch (ecp_mp::task::ECP_MP_main_error & e) {
				/* Obsluga bledow ECP_MP_main_error */
				if (e.error_class == lib::SYSTEM_ERROR)
					exit(EXIT_FAILURE);
			} /*end: catch */
			catch (mp::common::MP_main_error & e) {

				if (e.error_class == lib::SYSTEM_ERROR)
					exit(EXIT_FAILURE);

				/* Obsluga lib::NON_FATAL_ERROR */
				switch (e.error_no)
				{
					case ECP_ERRORS:
					case INVALID_ECP_PULSE_IN_MP_START_ALL:
					case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
					case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
						mp::common::mp_t->stop_and_terminate();
						break;
					case ECP_STOP_ACCEPTED:
						mp::common::mp_t->sr_ecp_msg->message("ecp STOP ACCEPTED");
						break;
					default:
						perror("Unidentified mp error");
						mp::common::mp_t->stop_and_terminate();
				}/*end:switch*/

			} /*end: catch */
			catch (mp::robot::MP_error & e) {
				if (e.error_class == lib::SYSTEM_ERROR) {
					exit(EXIT_FAILURE);
				}

				/* Obsluga lib::NON_FATAL_ERROR */
				switch (e.error_no)
				{
					case ECP_ERRORS:
					case INVALID_POSE_SPECIFICATION:
					case INVALID_COMMAND_TO_EDP:
					case EDP_ERROR:
					case INVALID_ROBOT_MODEL_TYPE:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
						break;
					default:
						perror("Unidentified mp error");
				}/*end:switch */
				mp::common::mp_t->stop_and_terminate();

			} /*end: catch*/

			catch (mp::generator::MP_error & e) {

				if (e.error_class == lib::SYSTEM_ERROR)
					exit(EXIT_FAILURE);

				/* Obsluga lib::NON_FATAL_ERROR*/
				switch (e.error_no)
				{
					case ECP_ERRORS:
					case INVALID_POSE_SPECIFICATION:
					case NON_EXISTENT_DIRECTORY:
					case NON_TRAJECTORY_FILE:
					case NON_EXISTENT_FILE:
					case READ_FILE_ERROR:
					case NON_COMPATIBLE_LISTS:
					case MAX_ACCELERATION_EXCEEDED:
					case MAX_VELOCITY_EXCEEDED:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, e.error_no);
						break;
					default:
						perror("Unidentified mp error");
				}/* end:switch*/
				mp::common::mp_t->stop_and_terminate();

			} /*end: catch*/

			catch (lib::sensor::sensor_error & e) {
				/* Wyswietlenie komunikatu. */
				mp::common::mp_t->sr_ecp_msg->message(e.error_class, e.error_no);
				printf("Mam blad czujnika section 2 (@%s:%d)\n", __FILE__, __LINE__);
			} /* end: catch sensor_error  */
			catch (ecp_mp::transmitter::transmitter_error & e) {
				/* Wyswietlenie komunikatu. */
				mp::common::mp_t->sr_ecp_msg->message(e.error_class, 0);
				printf("Mam blad trasnmittera section 2 (@%s:%d)\n", __FILE__, __LINE__);
			} /* end: catch sensor_error  */

			catch (const std::exception& e) {
				std::string tmp_string(" The following error has been detected: ");
				tmp_string += e.what();
				mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, tmp_string.c_str());
				std::cerr << "mp: The following error has been detected :\n\t" << e.what() << std::endl;
			}

			catch (...) { /* Dla zewnetrznej petli try*/
				/*   Wylapywanie niezdfiniowanych bledow  */
				/*  Komunikat o bledzie wysylamy do SR */
				mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
				exit(EXIT_FAILURE);
			} /*end: catch  */

		} // koniec: for(;;) - zewnetrzna petla

	} catch (...) { /* Dla zewnetrznej petli try*/
		/* Wylapywanie niezdefiniowanych bledow  */
		/* Komunikat o bledzie wysylamy do SR */
		printf("unexpected exception throw from catch section (@%s:%d)\n", __FILE__, __LINE__);
		mp::common::mp_t->sr_ecp_msg->message(lib::FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
		exit(EXIT_FAILURE);
	} /* end: catch  */

}
