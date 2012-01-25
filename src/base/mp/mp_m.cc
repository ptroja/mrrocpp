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

#include <boost/shared_ptr.hpp>

#include "base/ecp_mp/ecp_mp_exceptions.h"
#include "base/ecp_mp/transmitter.h"

#include "base/lib/mis_fun.h"

#include "mp_task.h"
#include "generator/mp_generator.h"
#include "mp_robot.h"
#include "mp_exceptions.h"

namespace mrrocpp {
namespace mp {
namespace common {

// obiekt z metodami i polami dla procesu MP (polimorficzny)
boost::shared_ptr <task::task> mp_t;

void catch_signal_in_mp(int sig)
{
	// print info message
	fprintf(stderr, "mp: %s\n", strsignal(sig));
	pid_t child_pid;
	int status;
	switch (sig)
	{
		case SIGTERM:
			if (mp_t) {
				mp_t->sr_ecp_msg->message("mp terminated");
			}
			// restore default (none) handler for SIGCHLD
			signal(SIGCHLD, SIG_DFL);
			exit(EXIT_SUCCESS);
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
}

} // namespace common
} // namespace mp
} // namespace mrrocpp

int main(int argc, char *argv[], char **arge)
{
	if (argc < 4) {
		std::cerr << "Usage: mp_m_c <ui_node_name> <mrrocpp_local_path> <config_file>" << std::endl;
		exit(EXIT_FAILURE);
	}

	// zewnetrzne try
	try {
		// This block is from where the configurator is supposed to be accessible
		boost::shared_ptr <lib::configurator> _config;

		try {
			_config = (boost::shared_ptr <lib::configurator>) new lib::configurator(argv[1], argv[2], lib::MP_SECTION);

			mp::common::mp_t = (boost::shared_ptr <mrrocpp::mp::task::task>) mp::task::return_created_mp_task(*_config);

			// Utworzenie listy robotow, powolanie procesow ECP i nawiazanie komunikacji z nimi
			mp::common::mp_t->create_robots();

			mp::common::mp_t->sr_ecp_msg->message("mp loaded");

			lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 4);

			signal(SIGTERM, &(mp::common::catch_signal_in_mp));

			signal(SIGCHLD, &(mp::common::catch_signal_in_mp));

			// ignore Ctrl-C signal, which comes from UI console
			signal(SIGINT, SIG_IGN);

		}

		catch (ecp_mp::exception::se & error) {
			exit(EXIT_FAILURE);
		}

		catch (mp::exception::se & error) {
			exit(EXIT_FAILURE);

		}

		catch (mp::exception::nfe & error) {

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			/* Obsluga lib::NON_FATAL_ERROR*/
			switch (error0)
			{
				case ECP_ERRORS:
				case INVALID_ECP_PULSE_IN_MP_START_ALL:
				case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
				case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
					if (mp::common::mp_t) {
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, error0);
					}
					break;
				default:
					perror("Message to SR has been already sent");
					break;
			}/* end:switch */
			if (mp::common::mp_t) {
				mp::common::mp_t->sr_ecp_msg->message("To terminate MP click STOP icon");
			}
			exit(EXIT_FAILURE);
		}

		catch (lib::exception::se_sensor & error) {

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			/* Wyswietlenie komunikatu. */
			if (mp::common::mp_t) {
				mp::common::mp_t->sr_ecp_msg->message(lib::SYSTEM_ERROR, error0);
			}
			printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}

		catch (lib::exception::fe_sensor & error) {

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			/* Wyswietlenie komunikatu. */
			if (mp::common::mp_t) {
				mp::common::mp_t->sr_ecp_msg->message(lib::FATAL_ERROR, error0);
			}
			printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}

		catch (ecp_mp::exception::se_tr & error) {
			/* Wyswietlenie komunikatu. */
			printf("Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}

		catch (const std::exception& e) {
			std::string tmp_string(" The following error has been detected: ");
			tmp_string += e.what();
			if (mp::common::mp_t) {
				mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, tmp_string.c_str());
			}
			std::cerr << "mp: The following error has been detected :\n\t" << e.what() << std::endl;
			exit(EXIT_FAILURE);
		}

		catch (...) { /* Dla zewnetrznej petli try*/
			/*   Wylapywanie niezdfiniowanych bledow  */
			/*  Komunikat o bledzie wysylamy do SR */
			if (mp::common::mp_t) {
				mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
			}
			exit(EXIT_FAILURE);
		}

		for (;;) { // Wewnetrzna petla nieskonczona

			try {
				mp::common::mp_t->sr_ecp_msg->message("mp - wcisnij start");

				// Oczekiwanie na zlecenie START od UI
				mp::common::mp_t->wait_for_start();

				// Wyslanie START do wszystkich ECP
				mp::common::mp_t->start_all();

				mp::common::mp_t->main_task_algorithm();

				// Oczekiwanie na STOP od UI
				mp::common::mp_t->wait_for_stop();

				// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
				mp::common::mp_t->terminate_all();
			} // end: try

			catch (ecp_mp::exception::se & error) {
				exit(EXIT_FAILURE);
			}

			catch (mp::exception::se & error) {
				exit(EXIT_FAILURE);

			}

			catch (mp::exception::nfe & error) {

				uint64_t error0 = 0;

				if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
					error0 = *tmp;
				}

				/* Obsluga lib::NON_FATAL_ERROR */
				switch (error0)
				{
					case ECP_ERRORS:
					case INVALID_ECP_PULSE_IN_MP_START_ALL:
					case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
					case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, error0);
						mp::common::mp_t->stop_and_terminate();
						break;
					case ECP_STOP_ACCEPTED:
						mp::common::mp_t->sr_ecp_msg->message("ecp STOP ACCEPTED");
						break;
					default:
						perror("Unidentified mp error");
						mp::common::mp_t->stop_and_terminate();
						break;
				}/*end:switch*/
			}

			catch (mp::exception::nfe_r & error) {
				uint64_t error0 = 0;

				if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
					error0 = *tmp;
				}
				/* Obsluga lib::NON_FATAL_ERROR */
				switch (error0)
				{
					case ECP_ERRORS:
					case INVALID_POSE_SPECIFICATION:
					case INVALID_COMMAND_TO_EDP:
					case EDP_ERROR:
					case INVALID_ROBOT_MODEL_TYPE:
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, error0);
						break;
					default:
						perror("Unidentified mp error");
						break;
				}/*end:switch */
				mp::common::mp_t->stop_and_terminate();

			} /*end: catch*/

			catch (mp::exception::nfe_g & error) {
				uint64_t error0 = 0;

				if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
					error0 = *tmp;
				}
				/* Obsluga lib::NON_FATAL_ERROR*/
				switch (error0)
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
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, error0);
						break;
					default:
						perror("Unidentified mp error");
				}/* end:switch*/
				mp::common::mp_t->stop_and_terminate();

			} /*end: catch*/

			catch (lib::exception::se_sensor & error) {

				uint64_t error0 = 0;

				if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
					error0 = *tmp;
				}

				/* Wyswietlenie komunikatu. */
				if (mp::common::mp_t) {
					mp::common::mp_t->sr_ecp_msg->message(lib::SYSTEM_ERROR, error0);
				}
				printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
			}

			catch (lib::exception::fe_sensor & error) {

				uint64_t error0 = 0;

				if (uint64_t const * tmp = boost::get_error_info <lib::exception::mrrocpp_error0>(error)) {
					error0 = *tmp;
				}

				/* Wyswietlenie komunikatu. */
				if (mp::common::mp_t) {
					mp::common::mp_t->sr_ecp_msg->message(lib::FATAL_ERROR, error0);
				}
				printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
			}

			catch (ecp_mp::exception::se_tr & error) {
				/* Wyswietlenie komunikatu. */
				printf("Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);

			}

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
