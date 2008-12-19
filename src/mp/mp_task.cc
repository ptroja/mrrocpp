// -------------------------------------------------------------------------
//                              mp_task.cc
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>


#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "mp/mp.h"
#include "mp/mp_r_conveyor.h"
#include "mp/mp_r_irp6_on_track.h"
#include "mp/mp_r_irp6_postument.h"
#include "mp/mp_r_irp6_mechatronika.h"
#include "mp/mp_r_speaker.h"
#include "mp/mp_common_generators.h"
#include "mp/mp_delay_ms_condition.h"

using namespace std;

// obsluga sygnalu
void mp_task::catch_signal_in_mp_task(int sig)
{
	printf("catch_signal_in_mp\n");
	pid_t child_pid;
	int status;
	switch (sig) {
		case SIGTERM:
		//case SIGINT:
			kill_all_ECP(robot_m);
			kill_all_VSP(sensor_m);
			printf("catch_signal_in_mp\n");
#if defined(__QNXNTO__)
			flushall();
#endif
			sr_ecp_msg->message("MP terminated");
			_exit(EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in MP process %s\n", config.section_name);
			signal(SIGSEGV, SIG_DFL);
			break;
		case SIGCHLD:
			printf("MP waitpid(...)"); fflush(stdout);
		   child_pid = waitpid(-1, &status, /*WNOHANG*/ WEXITED);
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
			   break;   if (child_pid == -1) {
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

}

#if !defined(USE_MESSIP_SRR)
name_attach_t* mp_task::mp_trigger_attach = NULL;
name_attach_t* mp_task::mp_attach = NULL;
#else
messip_channel_t* mp_task::mp_trigger_attach = NULL;
messip_channel_t* mp_task::mp_attach = NULL;
#endif

// mapa wszystkich robotow z iteratorem
map <ROBOT_ENUM, mp_robot*> mp_task::robot_m;

// KONSTRUKTORY
mp_task::mp_task(configurator &_config) : ecp_mp_task(_config)
{
	// dla scheduler'a
	all_gen_sets_waiting_for_ECP_pulse = false;
	ui_new_pulse = false;
}

mp_task::~mp_task()
{
	delete[] mrrocpp_network_path;
}


void mp_task::stop_and_terminate()
{
	sr_ecp_msg->message("To terminate MP click STOP icon");
	wait_for_stop (MP_EXIT);
	terminate_all (robot_m);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
bool mp_task::create_robots()
{
	/*
	 * this is necessary to first create robot and then assign it to robot_m
	 * reason: mp_robot() constructor uses this map (by calling
	 * mp_task::mp_wait_for_name_open_ecp_pulse() so needs the map to be in
	 * a consistent state
	 */
	mp_robot* created_robot;

	// ROBOT IRP6_ON_TRACK
	if (config.return_int_value("is_irp6_on_track_active", "[ui]")) {
		created_robot = new mp_irp6_on_track_robot (*this);
		robot_m[ROBOT_IRP6_ON_TRACK] = created_robot;
	}

	// ROBOT IRP6_POSTUMENT
	if (config.return_int_value("is_irp6_postument_active", "[ui]")) {
		created_robot = new mp_irp6_postument_robot (*this);
		robot_m[ROBOT_IRP6_POSTUMENT] = created_robot;
	}

	// ROBOT CONVEYOR
	if (config.return_int_value("is_conveyor_active", "[ui]")) {
		created_robot = new mp_conveyor_robot (*this);
		robot_m[ROBOT_CONVEYOR] = created_robot;
	}

	// ROBOT SPEAKER
	if (config.return_int_value("is_speaker_active", "[ui]")) {
		created_robot = new mp_speaker_robot (*this);
		robot_m[ROBOT_SPEAKER] = created_robot;
	}

	// ROBOT IRP6_MECHATRONIKA
	if (config.return_int_value("is_irp6_mechatronika_active", "[ui]")) {
		created_robot = new mp_irp6_mechatronika_robot (*this);
		robot_m[ROBOT_IRP6_MECHATRONIKA] = created_robot;
	}

	// ROBOT_ELECTRON
	if (config.return_int_value("is_electron_robot_active", "[ui]")) {
		created_robot = new mp_robot (ROBOT_ELECTRON, "[ecp_electron]", *this);
		robot_m[ROBOT_ELECTRON] = created_robot;
	}

	// ROBOT_SPEECHRECOGNITION
	if (config.return_int_value("is_speechrecognition_active", "[ui]")) {
		created_robot = new mp_robot (ROBOT_SPEECHRECOGNITION, "[ecp_speechrecognition]", *this);
		robot_m[ROBOT_SPEECHRECOGNITION] = created_robot;
	}

	// ROBOT_FESTIVAL
	if (config.return_int_value("is_festival_active", "[ui]")) {
		created_robot = new mp_robot (ROBOT_FESTIVAL, "[ecp_festival]", *this);
		robot_m[ROBOT_FESTIVAL] = created_robot;
	}

	return true;
}


// methods for mp template to redefine in concrete classes
void mp_task::task_initialization(void)
{
	sr_ecp_msg->message("MP loaded");
}

void mp_task::main_task_algorithm(void)
{}

// metody do obslugi najczesniej uzywanych generatorow
void mp_task::set_next_playerpos_goal (ROBOT_ENUM robot_l, const playerpos_goal_t &goal)
{
	// setting the next ecps state
	mp_set_next_ecps_state_generator mp_snes_gen(*this);

	mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];

	mp_snes_gen.configure(goal);

	mp_snes_gen.Move();
}

// metody do obslugi najczesniej uzywanych generatorow
void mp_task::set_next_ecps_state (int l_state, int l_variant, const char* l_string, int number_of_robots, ... )
{
	// setting the next ecps state
	mp_set_next_ecps_state_generator mp_snes_gen (*this);

	va_list arguments;                     // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_snes_gen.configure (l_state, l_variant, l_string);

	mp_snes_gen.Move();
}

// delay MP replacement
void mp_task::wait_ms (int _ms_delay) // zamiast delay
{
	mp_delay_ms_condition mp_ds_ms (*this, _ms_delay);

	mp_ds_ms.Move();
}

// send_end_motion
void mp_task::send_end_motion_to_ecps (int number_of_robots, ... )
{
	mp_send_end_motion_to_ecps_generator mp_semte_gen (*this);

	va_list arguments;                     // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_semte_gen.Move();
}

// send_end_motion
void mp_task::send_end_motion_to_ecps (int number_of_robots, ROBOT_ENUM *properRobotsSet)
{
	mp_send_end_motion_to_ecps_generator mp_semte_gen (*this);

	ROBOT_ENUM robot_l;

	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = properRobotsSet[x]; // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}

	mp_semte_gen.Move();
}

void mp_task::run_ext_empty_gen (bool activate_trigger, int number_of_robots, ... )
{
	mp_extended_empty_generator mp_ext_empty_gen (*this);

	va_list arguments;                     // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_ext_empty_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_ext_empty_gen.configure (activate_trigger);

	mp_ext_empty_gen.Move();
}

void mp_task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... )
{
	// CZYNNOSCI WSTEPNE
	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	map <ROBOT_ENUM, mp_robot*> robots_to_move, robots_to_wait_for_task_termination;
	map <ROBOT_ENUM, mp_robot*> robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;
	map <ROBOT_ENUM, mp_robot*>::iterator robots_map_iter;

	// powolanie generatora i jego konfiguracja
	mp_extended_empty_generator mp_ext_empty_gen (*this);
	mp_ext_empty_gen.configure (false);

	// na podstawie argumentow wywolania biezacej metody
	va_list arguments;    // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
	va_start ( arguments, number_of_robots_to_wait_for_task_termin);
	// najpierw zbior robots_to_move...
	for ( int x = 0; x < number_of_robots_to_move; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.

		if (robot_m.count(robot_l) == 0)
		{
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty");
		} else {
			robots_to_move[robot_l] = robot_m[robot_l];
		}
	}
	// ...potem zbior robots_to_wait_for_task_termination
	for ( int x = 0; x < number_of_robots_to_wait_for_task_termin; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		if (robot_m.count(robot_l) == 0)
		{
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
		} else {
			robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
		}
	}
	va_end ( arguments );              // Cleans up the list

	// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination.begin();
	        robot_m_iterator != robots_to_wait_for_task_termination.end(); robot_m_iterator++) {

		robots_map_iter = robots_to_move.find(robot_m_iterator->first);
		if (robots_map_iter == robots_to_move.end()) {
			sr_ecp_msg->message (SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
			throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		}
	}

	// GLOWNA PETLA

	do {
		// aktualizacja ziorow robotow i sprawdzenie czy zbior robots_to_wait_for_task_termination nie jest juz pusty
		// wtedy wyjscie z petli

		//	if (debug_tmp) printf(" run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
		// przygotowanie zapasowych list robotow
		robots_to_move_tmp.clear();
		robots_to_wait_for_task_termination_tmp.clear();

		robots_to_move_tmp = robots_to_move;
		robots_to_wait_for_task_termination_tmp = robots_to_wait_for_task_termination;

		// sprawdzenie zbioru robots_to_move
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_move_tmp.begin();
		        robot_m_iterator != robots_to_move_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
				robots_to_move.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie zbioru robots_to_wait_for_task_termination
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination_tmp.begin();
		        robot_m_iterator != robots_to_wait_for_task_termination_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("2 ");
				robots_to_wait_for_task_termination.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
		// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
		if (robots_to_wait_for_task_termination.empty())
			break;

		// przypisanie generatorowi mp_ext_empty_gen zbioru robots_to_move
		mp_ext_empty_gen.robot_m.clear();
		mp_ext_empty_gen.robot_m = robots_to_move;

		//	if (debug_tmp) printf("PRZED MOVE run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
		// uruchomienie generatora
		mp_ext_empty_gen.Move();
		//		if (debug_tmp) printf("ZA MOVE move run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
	} while (true);
	// koniec petli


}

void mp_task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ROBOT_ENUM *robotsToMove, ROBOT_ENUM *robotsWaitingForTaskTermination)
{
	// CZYNNOSCI WSTEPNE
	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	map <ROBOT_ENUM, mp_robot*> robots_to_move, robots_to_wait_for_task_termination;
	map <ROBOT_ENUM, mp_robot*> robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;
	map <ROBOT_ENUM, mp_robot*>::iterator robots_map_iter;

	// powolanie generatora i jego konfiguracja
	mp_extended_empty_generator mp_ext_empty_gen (*this);
	mp_ext_empty_gen.configure (false);

	// na podstawie argumentow wywolania biezacej metody
	//va_list arguments;    // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
	//va_start ( arguments, number_of_robots_to_wait_for_task_termin);
	// najpierw zbior robots_to_move...
	for ( int x = 0; x < number_of_robots_to_move; x++ )        // Loop until all numbers are added
	{
		robot_l = robotsToMove[x]; // Adds the next value in argument list to sum.

		if (robot_m.count(robot_l) == 0)
		{
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty");
		} else {
			robots_to_move[robot_l] = robot_m[robot_l];
		}
	}
	// ...potem zbior robots_to_wait_for_task_termination
	for ( int x = 0; x < number_of_robots_to_wait_for_task_termin; x++ )        // Loop until all numbers are added
	{
		robot_l = robotsWaitingForTaskTermination[x]; // Adds the next value in argument list to sum.
		if (robot_m.count(robot_l) == 0)
		{
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
		} else {
			robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
		}
	}
	//va_end ( arguments );              // Cleans up the list

	// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination.begin();
	        robot_m_iterator != robots_to_wait_for_task_termination.end(); robot_m_iterator++) {

		robots_map_iter = robots_to_move.find(robot_m_iterator->first);
		if (robots_map_iter == robots_to_move.end()) {
			sr_ecp_msg->message (SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
			throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		}
	}

	// GLOWNA PETLA

	do {
		// aktualizacja ziorow robotow i sprawdzenie czy zbior robots_to_wait_for_task_termination nie jest juz pusty
		// wtedy wyjscie z petli

		//	if (debug_tmp) printf(" run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
		// przygotowanie zapasowych list robotow
		robots_to_move_tmp.clear();
		robots_to_wait_for_task_termination_tmp.clear();

		robots_to_move_tmp = robots_to_move;
		robots_to_wait_for_task_termination_tmp = robots_to_wait_for_task_termination;

		// sprawdzenie zbioru robots_to_move
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_move_tmp.begin();
		        robot_m_iterator != robots_to_move_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
				robots_to_move.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie zbioru robots_to_wait_for_task_termination
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination_tmp.begin();
		        robot_m_iterator != robots_to_wait_for_task_termination_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("2 ");
				robots_to_wait_for_task_termination.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
		// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
		if (robots_to_wait_for_task_termination.empty())
			break;

		// przypisanie generatorowi mp_ext_empty_gen zbioru robots_to_move
		mp_ext_empty_gen.robot_m.clear();
		mp_ext_empty_gen.robot_m = robots_to_move;

		//	if (debug_tmp) printf("PRZED MOVE run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
		// uruchomienie generatora
		mp_ext_empty_gen.Move();
		//		if (debug_tmp) printf("ZA MOVE move run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
	} while (true);
	// koniec petli

}


// -------------------------------------------------------------------
// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
// -------------------------------------------------------------------

int mp_task::mp_receive_pulse (mp_receive_pulse_struct_t* outputs, MP_RECEIVE_PULSE_MODE tryb)
{

	struct sigevent event;
	int wyjscie = 0;

	event.sigev_notify = SIGEV_UNBLOCK;

	while (!wyjscie) {

		if (tryb == WITH_TIMEOUT) {
			TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE,  &event, NULL, NULL );// by Y zamiast creceive
		}

		outputs->rcvid = MsgReceive (mp_trigger_attach->chid, &(outputs->pulse_msg), sizeof(_pulse_msg), &(outputs->msg_info));

		if (outputs->rcvid == -1) {/* Error condition, exit */

			outputs->e = errno;
			wyjscie++;
			continue;
		}

		if (outputs->rcvid == 0) {/* Pulse received */

			switch (outputs->pulse_msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					printf("mp_receive_pulse puls _PULSE_CODE_DISCONNECT\n");
					/*
					 * A client disconnected all its connections (called
					 * name_close() for each name_open() of our name) or
					 * terminated
					 */
					ConnectDetach(outputs->pulse_msg.hdr.scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					//	  printf("MP_TRIGGER puls _PULSE_CODE_UNBLOCK\n");
					/*
					 * REPLY blocked client wants to unblock (was hit by
					 * a signal or timed out).  It's up to you if you
					 * reply now or later.
					 */
					break;
				default:
					// 	printf("MP_TRIGGER puls default: %d\n", outputs->pulse_msg.hdr.code);
					/*
					* A pulse sent by one of your processes or a
					* _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
					* from the kernel?
					*/
					break;
			}

			wyjscie++;
			continue;
		}

		if (outputs->rcvid > 0) {
			/* A QNX IO message received, reject */
			// ECP wywolalo name_open
			if (outputs->pulse_msg.hdr.type >= _IO_BASE && outputs->pulse_msg.hdr.type <= _IO_MAX) {
				// 	  printf("w MP_TRIGGER _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
				//  MsgError(rcvid, ENOSYS);
				//			  printf("mp_receive_ecp_pulse_return_t name_open: %d, %d\n", info.pid, info.scoid);
				MsgReply (outputs->rcvid, EOK, 0, 0);
				wyjscie++;
				//		ret.rt = false;
				continue;
			}
			/* A message (presumable ours) received, handle */
			printf("mp_receive_ecp_pulse server receive strange message of type: %d\n", outputs->pulse_msg.data);
			MsgReply(outputs->rcvid, EOK, 0, 0);
		}
	}

	return outputs->rcvid;
}

int mp_task::check_and_optional_wait_for_new_pulse (mp_receive_pulse_struct_t* outputs,
        WAIT_FOR_NEW_PULSE_ENUM process_mode, MP_RECEIVE_PULSE_MODE desired_wait_mode)
{

	int ret;
	bool exit_from_while = false;
	bool desired_pulse_found = false;

	MP_RECEIVE_PULSE_MODE current_wait_mode = WITH_TIMEOUT;

	// checking of already registered pulses

	if ((process_mode == NEW_ECP_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
		        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
			if ((robot_m_iterator->second->new_pulse) && (!(robot_m_iterator->second->robot_new_pulse_checked))) {
				desired_pulse_found = true;
			}
		}
	}

	if ((process_mode == NEW_UI_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
		if (ui_new_pulse) {
			desired_pulse_found = true;
		}
	}

	// receiving new pulses

	while (!exit_from_while) {
		ret = mp_receive_pulse (outputs, current_wait_mode);

		if (ret == -1) {
			if (outputs->e != ETIMEDOUT) {
				// tu ma byc wyjatek
				printf ("Blad MsgReceive() na kanale ecp_pusle w receive_pending_pulses\n");
			} else {
				if ((desired_wait_mode == WITHOUT_TIMEOUT) && (!(desired_pulse_found))) {
					current_wait_mode = WITHOUT_TIMEOUT;
				} else {
					exit_from_while = true;
				}
				continue;
			}

		} else if (ret == 0) {
			//			printf("check_and_optional_wait_for_new_pulse ret == 0\n");
			// wstawiamy informacje o pulsie ktory przyszedl do innego robota
			for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
			        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
				if (outputs->pulse_msg.hdr.scoid == robot_m_iterator->second->scoid) {
					//					printf("check_and_optional_wait_for_new_pulse w ECP\n");
					robot_m_iterator->second->pulse_code = outputs->pulse_msg.hdr.code;
					robot_m_iterator->second->new_pulse = true;
					if ((process_mode == NEW_ECP_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
						if (!(robot_m_iterator->second->robot_new_pulse_checked)) {
							desired_pulse_found = true;
							if (current_wait_mode == WITHOUT_TIMEOUT) {
								exit_from_while = true;
							}
						}
					}
					continue;
				}
			}

			if (outputs->pulse_msg.hdr.scoid == ui_scoid) {
				ui_pulse_code = outputs->pulse_msg.hdr.code;
				ui_new_pulse = true;
				if ((process_mode == NEW_UI_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
					desired_pulse_found = true;
					if (current_wait_mode == WITHOUT_TIMEOUT) {
						exit_from_while = true;
					}
				}
				continue;
			}

		} else if (ret > 0) {
			// jesli wlasciwy proces zrobil name_open
		}

	}

	if (desired_pulse_found)
		ret = 0;

	return ret;
}


int mp_task::mp_wait_for_name_open_ecp_pulse(mp_receive_pulse_struct_t* outputs)
{

	int ret;
	bool wyjscie = false;

	while (!wyjscie) {
		ret = mp_receive_pulse (outputs, WITHOUT_TIMEOUT);
		// jakis inny robot wyslal puls
		if (ret == -1) {
			// tu ma byc wyjatek
			printf ("Blad MsgReceive() na kanale ecp_pusle w mp_wait_for_name_open_ecp_pulse\n");
		} else if (ret == 0) {

			// wstawiamy informacje o pulsie ktory przyszedl do innego robota
			for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
			        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
				if (outputs->pulse_msg.hdr.scoid == robot_m_iterator->second->scoid) {
					robot_m_iterator->second->pulse_code = outputs->pulse_msg.hdr.code;
					robot_m_iterator->second->new_pulse = true;
					continue;
				}
			}

			if (outputs->pulse_msg.hdr.scoid == ui_scoid) {
				ui_pulse_code = outputs->pulse_msg.hdr.code;
				ui_new_pulse = true;
				continue;
			}

		} else if (ret > 0) {
			// zakladamy ze wlasciwy proces zrobi name_open
			wyjscie = true;

			/*
			// jesli wlasciwy proces zrobil name_open
			if ((ND_NODE_CMP(outputs->msg_info.nd, nd) == 0)&&(outputs->msg_info.pid == ECP_pid)) {
				wyjscie = true;
				continue;
			} else {
				printf ("niewlasciwy proces zrobil name_open na kanale ECP_PULSE\n");
			}
			*/
		}

	}

	return ret;
}



int mp_task::mp_wait_for_ui_name_open()
{

	int ret;
	mp_receive_pulse_struct_t outputs;
	bool wyjscie = false;

	while (!wyjscie) {
		ret = mp_receive_pulse (&outputs, WITHOUT_TIMEOUT);
		// jakis inny robot wyslal puls
		if (ret == -1) {
			// tu ma byc wyjatek
			printf ("Blad MsgReceive() na kanale ecp_pusle w mp_wait_for_name_open_ecp_pulse\n");
		} else if (ret == 0) {
			printf ("Blad mp_wait_for_ui_name_open ret = 0\n");
		} else if (ret > 0) {
			// jesli wlasciwy proces zrobil name_open
			ui_scoid = outputs.msg_info.scoid;
			wyjscie = true;
			continue;
		}
	}

	return ret;
}

// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE

void mp_task::mp_receive_ui_or_ecp_pulse (map <ROBOT_ENUM, mp_robot*>& _robot_m, mp_generator& the_generator )
{

	enum MP_STATE_ENUM
	{
	    MP_STATE_RUNNING,
	    MP_STATE_PAUSED
	};

	MP_STATE_ENUM mp_state = MP_STATE_RUNNING;
	int rcvid;
	mp_receive_pulse_struct_t input;

	bool ui_exit_from_while = false;
	bool ecp_exit_from_while = (the_generator.wait_for_ECP_pulse) ? false : true;

	while (!(ui_exit_from_while && ecp_exit_from_while)) {

		if (mp_state == MP_STATE_RUNNING){
				rcvid = check_and_optional_wait_for_new_pulse (
						&input, NEW_UI_OR_ECP_PULSE,
						ecp_exit_from_while ? WITH_TIMEOUT : WITHOUT_TIMEOUT);
		} else {
			rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);
		}

		if (rcvid == -1) {// Error condition
			if (mp_state == MP_STATE_RUNNING) {
				if (input.e != ETIMEDOUT) {// by Y zamiast creceive
					// Blad komunikacji miedzyprocesowej - wyjatek
					perror("Creceive STOP or PAUSE proxy from UI failed ?");
					sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP:Creceive STOP pulse from UI failed");
					throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
				} else {
					ui_exit_from_while = true;
					continue;
				}
			} else if (mp_state == MP_STATE_PAUSED) {
				perror("Creceive RESUME proxy from UI failed ?\n");
				sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP: receive RESUME pulse from UI failed");
				throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
			}
		} else if (rcvid == 0) {
			if (ui_new_pulse) {

				ui_new_pulse = false;

				if (ui_pulse_code == MP_STOP) {
					terminate_all (_robot_m);
					throw MP_main_error(NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
					//return true;
				}

				if (ui_pulse_code == MP_PAUSE) {
					mp_state = MP_STATE_PAUSED;
					ui_exit_from_while = false;
					continue;
				}

				if (mp_state == MP_STATE_PAUSED) {// oczekujemy na resume
					if (ui_pulse_code == MP_RESUME) { // odebrano resume
						mp_state = MP_STATE_RUNNING;
						ui_exit_from_while = true;
					}
				} else {
					if (ui_pulse_code == MP_TRIGGER) { // odebrano trigger
						ui_exit_from_while = true;
						the_generator.trigger = true;
					}
				}
				continue;
			} else {

				if (mp_state == MP_STATE_RUNNING) {
					ui_exit_from_while = true;
				}
			}

			if (the_generator.wait_for_ECP_pulse) {
				for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
				        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
					if ((robot_m_iterator->second->new_pulse) && (!(robot_m_iterator->second->robot_new_pulse_checked))) {
						robot_m_iterator->second->robot_new_pulse_checked = true;
						//	 if (debug_tmp)	printf("wait_for_ECP_pulse r: %d, pc: %d\n", robot_m_iterator->first, robot_m_iterator->second->pulse_code);
						ecp_exit_from_while = true;
					}
				}
			} else {
				ecp_exit_from_while = true;
			}
			continue;

		} else if (rcvid > 0) {
			fprintf(stderr, "MP_TRIGGER server receive strange message\n");
		}
	}
//	return false;

}

// funkcja odbierajaca pulsy z UI wykorzystywana w Move

bool mp_task::mp_receive_ui_pulse (map <ROBOT_ENUM, mp_robot*>& _robot_m, short* trigger )
{
	enum MP_STATE_ENUM
	{
	    MP_STATE_RUNNING,
	    MP_STATE_PAUSED
	};

	MP_STATE_ENUM mp_state = MP_STATE_RUNNING;

	bool ui_exit_from_while = false;

	while (!ui_exit_from_while) {
		mp_receive_pulse_struct_t input;
		int rcvid = check_and_optional_wait_for_new_pulse (
				&input, NEW_UI_PULSE,
				(mp_state == MP_STATE_RUNNING) ? WITH_TIMEOUT : WITHOUT_TIMEOUT);

		if (rcvid == -1) {// Error condition
			if (mp_state == MP_STATE_RUNNING) {
				if (input.e != ETIMEDOUT) {// by Y zamiast creceive
					// Blad komunikacji miedzyprocesowej - wyjatek
					perror("Creceive STOP or PAUSE proxy from UI failed ?");
					sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP:Creceive STOP pulse from UI failed");
					throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
				} else {
					ui_exit_from_while = true;
					continue;
				}
			} else if (mp_state == MP_STATE_PAUSED) {
				perror("Creceive RESUME proxy from UI failed ?\n");
				sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP: receive RESUME pulse from UI failed");
				throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
			}
		} else if (rcvid == 0) {
			if (ui_new_pulse) {
				ui_new_pulse = false;
				if (ui_pulse_code == MP_STOP) {
					terminate_all (_robot_m);
					return true;
				}

				if (ui_pulse_code == MP_PAUSE) {
					// printf("ui_new_pulse MP_PAUSED\n");
					mp_state = MP_STATE_PAUSED;
				}

				if (mp_state == MP_STATE_PAUSED) {// oczekujemy na resume
					if (ui_pulse_code == MP_RESUME) { // odebrano resume
						mp_state = MP_STATE_RUNNING;
						ui_exit_from_while = true;
					}
				} else {
					if (ui_pulse_code == MP_TRIGGER) { // odebrano trigger
						ui_exit_from_while = true;
						*trigger = true;
					}
				}
			}
			continue;

		} else if (rcvid > 0) {
			printf("MP_TRIGGER server receive strange message of type: \n");
		}
	}
	return false;
}

void mp_task::initialize_communication()
{
	char* sr_net_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	char* mp_attach_point =	config.return_attach_point_name(configurator::CONFIG_SERVER, "mp_attach_point");

	if (( sr_ecp_msg = new sr_ecp(MP, mp_attach_point, sr_net_attach_point)) == NULL) { // Obiekt do komuniacji z SR
		perror ( "Unable to locate SR\n");

		delete [] sr_net_attach_point;
		delete [] mp_attach_point;

		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	delete [] sr_net_attach_point;


	// Rejestracja procesu MP
#if !defined(USE_MESSIP_SRR)
	if ((mp_attach = name_attach(NULL, mp_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
#else
	if ((mp_attach = messip_channel_create(NULL, mp_attach_point, MESSIP_NOTIMEOUT, 0)) == NULL) {
#endif
		uint64_t e = errno; // kod bledu systemowego
		perror("Failed to attach Master Process\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "MP: Failed to name attach");

		delete [] mp_attach_point;

		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	delete [] mp_attach_point;



	char* mp_pulse_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "mp_pulse_attach_point");

	// Rejestracja kanalu dla pulsow start z procesu UI i ECP
#if !defined(USE_MESSIP_SRR)
	if (( mp_trigger_attach = name_attach(NULL, mp_pulse_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
#else
	if ((mp_attach = messip_channel_create(NULL, mp_pulse_attach_point, MESSIP_NOTIMEOUT, 0)) == NULL) {
#endif
		uint64_t e = errno; // kod bledu systemowego
		perror("Failed to attach UI Pulse chanel for Master Process\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "MP: Failed to name attach  UI Pulse");

		delete [] mp_pulse_attach_point;

		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	delete [] mp_pulse_attach_point;



	mp_wait_for_ui_name_open();
}
// -------------------------------------------------------------------



// ------------------------------------------------------------------------
bool mp_task::clear_gen_list ()
{
	if (!gen_list.empty()) {
		gen_list.clear();
	}

	return true;
}
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
bool mp_task::add_gen (mp_generator* gen_l)
{
	if (!gen_list.empty()) {
		gen_list.push_back(gen_l);
	} else {
		gen_list.push_back(gen_l);
		gen_list_iterator = gen_list.begin();
	}

	return true;
}
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------
#if 0		// not used any more (ptrojane)
bool mp_task::rm_gen (mp_generator* gen_l)
{
	if (!gen_list.empty()) {
		std::list<mp_generator*>::iterator gen_local_list_iterator = find(gen_list.begin(), gen_list.end(), gen_l);
		if (gen_local_list_iterator != gen_list.end()) {
			gen_list.erase(gen_local_list_iterator);
			if (!gen_list.empty()) {
				gen_local_list_iterator = gen_list.begin();
			}
			return true;
		}
	}

	return false;
}
#endif
// ------------------------------------------------------------------------








// ------------------------------------------------------------------------
bool mp_task::scheduler_run ()
{

	short trigger;

	// mp_receive_ecp_pulse_return_t ret;
	int ret;
	mp_receive_pulse_struct_t input;

	if (gen_list.empty()) {
		printf ("gen_list empty w scheduler_run\n");
		return true;
	}

	while (1) {

		switch ((*gen_list_iterator)->phase) {
			case mp_generator::BEFORE_FIRST_STEP:
				if (!((*gen_list_iterator)->first_step () )) {
					(*gen_list_iterator)->phase = mp_generator::GS_FINISHED;
					return false;
				}
				(*gen_list_iterator)->phase = mp_generator::AFTER_STEP;
				break;
			case mp_generator::AFTER_STEP:
				trigger = false;
				// ew. odbior pulsu
				if (mp_receive_ui_pulse (robot_m, &trigger) == true)
					return true;
				if (trigger) {
					for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
					        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
						(*gen_local_list_iterator)->trigger = trigger;
					}
				}
				all_sensors_initiate_reading ((*gen_list_iterator)->sensor_m);
				(*gen_list_iterator)->phase = mp_generator::AFTER_INITIATE_READING;
				break;
			case mp_generator::AFTER_INITIATE_READING:
				// odebranie wszystkich dostarczonych juz pulsow
				/*
				do
			{
					ret = mp_receive_ecp_pulse (WITH_TIMEOUT);
				//					if (ret.rcvid == 0) cout << "AFTER_INITIATE_READING rcvid 0" << endl;
		} while (ret.rcvid == 0);
				*/
				ret = check_and_optional_wait_for_new_pulse (&input, NEW_UI_OR_ECP_PULSE, WITH_TIMEOUT);
				// sprawdzamy czy wszystkie ECP sa gotowe do komunikacji
				for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = (*gen_list_iterator)->
				        robot_m.begin();
				        robot_m_iterator != (*gen_list_iterator)->robot_m.end();
				        robot_m_iterator++) {
					//jesli ktorykolwiek robot nie jest gotowy
					if ((robot_m_iterator->second->communicate) && (!((robot_m_iterator->second->new_pulse)
					        && ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
					            (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE))))) {
						(*gen_list_iterator)->phase = mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION;

						// sprawdzenie czy wszystkie pozostale gen_set'y tez sa w tym stanie
						all_gen_sets_waiting_for_ECP_pulse = true;
						for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
						        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
							// jesli ktorykolwiek generator nie jest jeszcze zawieszony
							if ((*gen_local_list_iterator)->phase != mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION) {
								all_gen_sets_waiting_for_ECP_pulse = false;
								break;
							}
						}
						break;
					}
				}

				// jesli wszytkie aktywne roboty dla biezacego generatore sa gotowe
				if ((*gen_list_iterator)
				        ->phase != mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION) {
					execute_all((*gen_list_iterator)->robot_m);
					(*gen_list_iterator)->phase = mp_generator::AFTER_EXECUTE_MOTION;
				}
				(*gen_list_iterator)->new_pulse_checked = false;

				break;
			case mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION:
				// jesli wszystkie gen_set'y czekaja na puls z ECP
				if (all_gen_sets_waiting_for_ECP_pulse) {
					//			cout << "WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION all_gen_sets_waiting_for_ECP_pulse" << endl;
					if ((*gen_list_iterator)->new_pulse_checked) {
						// odbieramy puls z ECP z zawieszaniem bo nie mamy zadnego pulsu, ktory by oczekiwal
						// ret = mp_receive_ecp_pulse (WITHOUT_TIMEOUT);
						ret = check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);

						for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
						        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
							if ((robot_m_iterator->second->communicate) && ((robot_m_iterator->second->new_pulse)
							        && ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
							            (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE)))) {
								robot_m_iterator->second->robot_new_pulse_checked = true;
							}
						}

						// oznaczamy ze pozostale generatory nie zbadaly jeszcze biezacego pulsu od ECP
						for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
						        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
							(*gen_local_list_iterator)->new_pulse_checked = false;
						}

					}
					(*gen_list_iterator)->new_pulse_checked = true;
				} else {
					// odebranie wszystkich dostarczonych juz pulsow

					ret = check_and_optional_wait_for_new_pulse (&input, NEW_UI_OR_ECP_PULSE, WITH_TIMEOUT);
				}

				(*gen_list_iterator)->phase = mp_generator::AFTER_EXECUTE_MOTION;
				// sprawdzamy czy wszystkie ECP sa gotowe do komunkacji
				for (map <ROBOT_ENUM, mp_robot*>
				        ::iterator robot_m_iterator = (*gen_list_iterator)->robot_m.begin();
				        robot_m_iterator != (*gen_list_iterator)->robot_m.end();
				        robot_m_iterator++) {
					if ((robot_m_iterator->second->communicate) && (!((robot_m_iterator->second->new_pulse)
					        && ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
					            (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE))))) {
						(*gen_list_iterator)->phase = mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION;
						break;
					}
				}

				// jesli wszytkie aktywne roboty dla biezacego generatore sa gotowe
				if ((*gen_list_iterator)
				        ->phase != mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION) {
					execute_all((*gen_list_iterator)->robot_m);
					(*gen_list_iterator)->phase = mp_generator::AFTER_EXECUTE_MOTION;
					all_gen_sets_waiting_for_ECP_pulse = false;
				}

				break;
			case mp_generator::AFTER_EXECUTE_MOTION:
				all_sensors_get_reading((*gen_list_iterator)->sensor_m);
				(*gen_list_iterator)->phase = mp_generator::AFTER_GET_READING;
				break;
			case mp_generator::AFTER_GET_READING:
				if (!((*gen_list_iterator)->next_step ())
				   ) {
					(*gen_list_iterator)->phase = mp_generator::GS_FINISHED;
					return false;
				}
				(*gen_list_iterator)->phase = mp_generator::AFTER_STEP;
				break;
			case mp_generator::GS_FINISHED:
				trigger = false;
				// ew. odbior pulsu
				if (mp_receive_ui_pulse (robot_m, &trigger)
				   ) return true;
				if (trigger) {
					for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
					        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
						(*gen_local_list_iterator)->trigger = trigger;
					}
				}
				break;
			default:

				break;
		}

		// aktualizacja wskaznika na biezacy generator_set
		if (!((++gen_list_iterator) != gen_list.end()))
			gen_list_iterator =  gen_list.begin();
	}

	return true;
}
// ------------------------------------------------------------------------

void mp_task::wait_for_start ()
{
	// Oczekiwanie na zlecenie START od UI

	while (1) {
		mp_receive_pulse_struct_t input;
		check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);
		if (ui_new_pulse) {
			ui_new_pulse = false;
			if (ui_pulse_code == MP_START) {
				break;
			}
		}
	}

	sr_ecp_msg->message("MP user program is running");
}
// ------------------------------------------------------------------------


void mp_task::wait_for_stop (WAIT_FOR_STOP_ENUM tryb)
{
	// Oczekiwanie na zlecenie STOP od UI

	sr_ecp_msg->message("To terminate user program click STOP icon");
	bool wyjscie = false;

	while (!wyjscie) {
		mp_receive_pulse_struct_t input;
		int rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);

		if (rcvid == -1)/* Error condition, exit */
		{
			if (input.e != ETIMEDOUT)
			{
				perror("Receive StopProxy failed (MP)");
				sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP: Receive StopProxy failed");
				switch (tryb) {
					case MP_THROW:
						throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
						break;
					case MP_EXIT:
						exit(EXIT_FAILURE);
						break;
					default:
						printf("bledny tryb w wait_for_stop\n");
						break;
				}
			}
		} else {
			// if UI pulse occured
			if (ui_new_pulse) {
				ui_new_pulse = false;
				if (ui_pulse_code == MP_STOP) {
					wyjscie = true;
					continue;
				}
			}
		}
	}
}
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------

void mp_task::start_all (map <ROBOT_ENUM, mp_robot*>& _robot_m)
{
	// Wystartowanie wszystkich ECP

	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator;
	map <ROBOT_ENUM, mp_robot*> robots_m_tmp, robots_m_tmp2;

	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	robots_m_tmp = _robot_m;

	while (!(robots_m_tmp.empty())) {

		// przygotowanie wersji tymczasowej do usuwania robotow
		robots_m_tmp2.clear();

		for (robot_m_iterator = robots_m_tmp.begin(); robot_m_iterator != robots_m_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->new_pulse ) {
				if (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_START) {
					robot_m_iterator->second->new_pulse = false;
					robot_m_iterator->second->robot_new_pulse_checked = false;
					robot_m_iterator->second->start_ecp();
				} else {
					printf("phase 2 bledny kod pulsu w start_all\n");
					throw MP_main_error(NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_START_ALL);
				}
			} else {
				// dodaj robota do listy jeszcze nie obsluzonych
				robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
			}
		}

		// ponowne przepisanie map
		robots_m_tmp = robots_m_tmp2;

		if (!(robots_m_tmp.empty())) {
			mp_receive_pulse_struct_t input;
			check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
		}
	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void mp_task::execute_all (map <ROBOT_ENUM, mp_robot*>& _robot_m)
{
	// Wystartowanie wszystkich ECP
	// do przepisania wg http://www.thescripts.com/forum/thread62378.html

	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator;
	map <ROBOT_ENUM, mp_robot*> robots_m_tmp, robots_m_tmp2;

	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	for (robot_m_iterator = _robot_m.begin(); robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		if (robot_m_iterator->second->communicate) {
			robots_m_tmp[robot_m_iterator->first] = robot_m_iterator->second;
		}
	}

	while (!(robots_m_tmp.empty())) {

		// przygotowanie wersji tymczasowej do usuwania robotow
		robots_m_tmp2.clear();

		for (robot_m_iterator = robots_m_tmp.begin(); robot_m_iterator != robots_m_tmp.end(); robot_m_iterator++) {
			// komunikujemy sie tylko z aktywnymi robotami
			if (robot_m_iterator->second->new_pulse) {
				if ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
				        (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE)) {
					robot_m_iterator->second->new_pulse = false;
					robot_m_iterator->second->robot_new_pulse_checked = false;
					robot_m_iterator->second->execute_motion();
				} else {
					throw MP_main_error(NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL);
				}
			} else {
				// dodaj robota do listy jeszcze nie obsluzonych
				robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
			}
		}

		// ponowne przepisanie map
		robots_m_tmp = robots_m_tmp2;

		if (!(robots_m_tmp.empty())) {
			mp_receive_pulse_struct_t input;
			check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
		}
	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void mp_task::terminate_all (map <ROBOT_ENUM, mp_robot*>& _robot_m)
{
	// Zatrzymanie wszystkich ECP
	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator;

	map <ROBOT_ENUM, mp_robot*> robots_m_tmp, robots_m_tmp2;

	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	robots_m_tmp = _robot_m;

	while (!(robots_m_tmp.empty())) {

		// przygotowanie wersji tymczasowej do usuwania robotow
		robots_m_tmp2.clear();

		for (robot_m_iterator = robots_m_tmp.begin(); robot_m_iterator != robots_m_tmp.end(); robot_m_iterator++) {

			if (robot_m_iterator->second->new_pulse) {
				//if ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_STOP) || (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND))
				if (1) {
					robot_m_iterator->second->new_pulse = false;
					robot_m_iterator->second->robot_new_pulse_checked = false;
					robot_m_iterator->second->terminate_ecp();
				} else {
					printf("phase 2 bledny kod pulsu w terminate_all\n");
					throw MP_main_error(NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL);
				}
			} else {
				// dodaj robota do listy jeszcze nie obsluzonych
				robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
			}
		}

		// ponowne przepisanie map
		robots_m_tmp = robots_m_tmp2;

		if (!(robots_m_tmp.empty())) {
			mp_receive_pulse_struct_t input;
			check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
		}
	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void mp_task::kill_all_ECP (map <ROBOT_ENUM, mp_robot*>& _robot_m)
{
	// Zabicie wszystkich ECP z mapy
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
	        robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
#if defined(PROCESS_SPAWN_RSH)
		kill(robot_m_iterator->second->ECP_pid, SIGTERM);
#else
		SignalKill(robot_m_iterator->second->nd, robot_m_iterator->second->ECP_pid, 0, SIGTERM, 0, 0);
#endif
	}
}
// ------------------------------------------------------------------------




mp_sub_task::mp_sub_task(mp_task &_mp_t):
        mp_t(_mp_t)
{}

