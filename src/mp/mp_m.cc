// ------------------------------------------------------------------------
//
//                      MASTER PROCESS (MP) - main()
//
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "mp/mp.h"

// obiekt z metodami i polami dla procesu MP (polimorficzny)
mp_task* mp_t;

void catch_signal_in_mp(int sig)
{

	mp_t->catch_signal_in_mp_task(sig);
	delete mp_t;
	printf("za catch_signal_in_mp\n");
#if defined(__QNXNTO__)
	flushall();
#endif
}

int main (int argc, char *argv[], char **arge)
{
	printf("MP\n");
#if defined(__QNXNTO__)
	flushall();
#endif
	// zewnetrzne try
	try {

		if (argc < 6) {
			printf(" Usage: mp_m_c <ui_node_name> <mrrocpp_local_path> <config_file> <session_name>\n");
			exit(EXIT_FAILURE);
		}

		try	{
			configurator * _config = new configurator(argv[1], argv[2], argv[3], "[mp]", argv[5]);
#if defined(PROCESS_SPAWN_YRSH)
			if (argc>6) {
		 		_config->answer_to_y_rsh_spawn(argv[6]);
		 		signal(SIGINT, SIG_IGN);
		 	}
#endif
			mp_t = return_created_mp_task(*_config);

			set_thread_priority(pthread_self() , MAX_PRIORITY-4);
			signal(SIGTERM, &(catch_signal_in_mp));
			//signal(SIGINT,  &(catch_signal_in_mp));
			signal(SIGSEGV, &(catch_signal_in_mp));
			signal(SIGCHLD, &(catch_signal_in_mp));
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif

			mp_t->initialize_communication();

			// Utworzenie listy robotow, powolanie procesow ECP i nawiazanie komunikacji z nimi
			mp_t->create_robots();

			mp_t->task_initialization();
		}
		catch (ECP_MP_main_error e) {
			/* Obsluga bledow ECP_MP_main_error */
			if (e.error_class == SYSTEM_ERROR)
				exit(EXIT_FAILURE);
		} /*end: catch */
		catch (MP_main_error e) {

			perror("initialize incorrect");
			if (e.error_class == SYSTEM_ERROR)
				exit(EXIT_FAILURE);

			/* Obsluga NON_FATAL_ERROR*/
			switch (e.mp_error) {
				case ECP_ERRORS:
				case INVALID_ECP_PULSE_IN_MP_START_ALL:
				case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
				case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
					mp_t->sr_ecp_msg->message(NON_FATAL_ERROR, e.mp_error);
					break;
				default:
					perror("Message to SR has been already sent");
			}/* end:switch */
			mp_t->sr_ecp_msg->message("To terminate MP click STOP icon");
		} /*end: catch*/

		catch (sensor::sensor_error e) {
			/* Wyswietlenie komunikatu. */
			mp_t->sr_ecp_msg->message (e.error_class, e.error_no);
			printf("Mam blad czujnika section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */
		catch (transmitter::transmitter_error e) {
			/* Wyswietlenie komunikatu. */
			mp_t->sr_ecp_msg->message (e.error_class, e.error_no);
			printf("Mam blad trasnmittera section 1 (@%s:%d)\n", __FILE__, __LINE__);
		} /* end: catch sensor_error  */
		catch (...) {  /* Dla zewnetrznej petli try*/
			/*   Wylapywanie niezdfiniowanych bledow  */
			/*  Komunikat o bledzie wysylamy do SR */
			mp_t->sr_ecp_msg->message (NON_FATAL_ERROR, (uint64_t) MP_UNIDENTIFIED_ERROR);
			exit(EXIT_FAILURE);
		} /*end: catch  */

		for (;;) {  // Zewnetrzna petla nieskonczona

			try {
				mp_t->main_task_algorithm();
			}  // end: try

			catch (ECP_MP_main_error e) {
				/* Obsluga bledow ECP_MP_main_error */
				if (e.error_class == SYSTEM_ERROR)
					exit(EXIT_FAILURE);
			} /*end: catch */
			catch (MP_main_error e) {

				if (e.error_class == SYSTEM_ERROR)
					exit(EXIT_FAILURE);

				/* Obsluga NON_FATAL_ERROR */
				switch (e.mp_error) {
					case ECP_ERRORS:
					case INVALID_ECP_PULSE_IN_MP_START_ALL:
					case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
					case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
						mp_t->sr_ecp_msg->message(NON_FATAL_ERROR, e.mp_error);
						break;
					default:
						perror("Unidentified mp error");
				}/*end:switch*/
				mp_t->sr_ecp_msg->message("To terminate MP click STOP icon");

				mp_t->wait_for_stop (MP_EXIT);

				/* by Y*/
				mp_t->terminate_all (mp_t->robot_m);


			} /*end: catch */
			catch (mp_robot::MP_error e) {
				if (e.error_class == SYSTEM_ERROR) {
					exit(EXIT_FAILURE);
				}

				/* Obsluga NON_FATAL_ERROR */
				switch (e.mp_error) {
					case ECP_ERRORS:
					case INVALID_POSE_SPECIFICATION:
					case INVALID_ECP_COMMAND:
					case INVALID_COMMAND_TO_EDP:
					case EDP_ERROR:
					case INVALID_EDP_REPLY:
					case INVALID_RMODEL_TYPE:
						mp_t->sr_ecp_msg->message(NON_FATAL_ERROR, e.mp_error);
						break;
					default:
						perror("Unidentified mp error");
				}/*end:switch */
				mp_t->sr_ecp_msg->message("To terminate MP click STOP icon");

				mp_t->wait_for_stop (MP_EXIT);
				/* by Y*/

				mp_t->terminate_all (mp_t->robot_m);

			} /*end: catch*/

			catch (mp_generator::MP_error e) {

				if (e.error_class == SYSTEM_ERROR)
					exit(EXIT_FAILURE);

				/* Obsluga NON_FATAL_ERROR*/
				switch (e.mp_error) {
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
						mp_t->sr_ecp_msg->message(NON_FATAL_ERROR, e.mp_error);
						break;
					default:
						perror("Unidentified mp error");
				}/* end:switch*/
				mp_t->sr_ecp_msg->message("To terminate MP click STOP icon");

				mp_t->wait_for_stop (MP_EXIT);

				mp_t->terminate_all (mp_t->robot_m);

			} /*end: catch*/

			catch (sensor::sensor_error e) {
				/* Wyswietlenie komunikatu. */
				mp_t->sr_ecp_msg->message (e.error_class, e.error_no);
				printf("Mam blad czujnika section 2 (@%s:%d)\n", __FILE__, __LINE__);
			} /* end: catch sensor_error  */
			catch (transmitter::transmitter_error e) {
				/* Wyswietlenie komunikatu. */
				mp_t->sr_ecp_msg->message (e.error_class, e.error_no);
				printf("Mam blad trasnmittera section 2 (@%s:%d)\n", __FILE__, __LINE__);
			} /* end: catch sensor_error  */

			catch (...) {  /* Dla zewnetrznej petli try*/
				/*   Wylapywanie niezdfiniowanych bledow  */
				/*  Komunikat o bledzie wysylamy do SR */
				mp_t->sr_ecp_msg->message (NON_FATAL_ERROR, (uint64_t) MP_UNIDENTIFIED_ERROR);
				exit(EXIT_FAILURE);
			} /*end: catch  */

		} // koniec: for(;;) - zewnetrzna petla

	}
	catch (...) {  /* Dla zewnetrznej petli try*/
		/* Wylapywanie niezdfiniowanych bledow  */
		/* Komunikat o bledzie wysylamy do SR */
		printf("unexpected exception throw from catch section (@%s:%d)\n", __FILE__, __LINE__);
		mp_t->sr_ecp_msg->message (NON_FATAL_ERROR, (uint64_t) MP_UNIDENTIFIED_ERROR);
		exit(EXIT_FAILURE);
	} /* end: catch  */


}
