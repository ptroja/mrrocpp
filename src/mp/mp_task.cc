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
#include <string.h>


#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "mp/mp.h"
#include "mp/mp_r_conveyor.h"
#include "mp/mp_r_irp6_on_track.h"
#include "mp/mp_r_irp6_postument.h"
#include "mp/mp_r_irp6_mechatronika.h"
#include "mp/mp_r_speaker.h"
#include "mp/mp_r_polycrank.h"
#include "mp/mp_common_generators.h"
#include "mp/mp_delay_ms_condition.h"

namespace mrrocpp {
namespace mp {
namespace task {

using namespace std;

// obsluga sygnalu
void task::catch_signal_in_mp_task(int sig)
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
name_attach_t* task::mp_pulse_attach = NULL;
name_attach_t* task::mp_attach = NULL;
#else
messip_channel_t* task::mp_pulse_attach = NULL;
messip_channel_t* task::mp_attach = NULL;
#endif

// mapa wszystkich robotow z iteratorem
map <lib::ROBOT_ENUM, robot::robot*> task::robot_m;

// KONSTRUKTORY
task::task(lib::configurator &_config) : ecp_mp::task::task(_config)
{
	ui_new_pulse = false;
}

task::~task()
{
}


void task::stop_and_terminate()
{
	sr_ecp_msg->message("To terminate MP click STOP icon");
	wait_for_stop (common::MP_EXIT);
	terminate_all (robot_m);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void task::create_robots()
{
	/*
	 * this is necessary to first create robot and then assign it to robot_m
	 * reason: mp_robot() constructor uses this map (by calling
	 * mp_task::mp_wait_for_name_open() so needs the map to be in
	 * a consistent state
	 */
	robot::robot* created_robot;

	// ROBOT IRP6_ON_TRACK
	if (config.return_int_value("is_irp6_on_track_active", "[ui]")) {
		created_robot = new robot::irp6_on_track (*this);
		robot_m[lib::ROBOT_IRP6_ON_TRACK] = created_robot;
	}

	// ROBOT IRP6_POSTUMENT
	if (config.return_int_value("is_irp6_postument_active", "[ui]")) {
		created_robot = new robot::irp6_postument (*this);
		robot_m[lib::ROBOT_IRP6_POSTUMENT] = created_robot;
	}

	// ROBOT CONVEYOR
	if (config.return_int_value("is_conveyor_active", "[ui]")) {
		created_robot = new robot::conveyor (*this);
		robot_m[lib::ROBOT_CONVEYOR] = created_robot;
	}

	// ROBOT SPEAKER
	if (config.return_int_value("is_speaker_active", "[ui]")) {
		created_robot = new robot::speaker (*this);
		robot_m[lib::ROBOT_SPEAKER] = created_robot;
	}

	// ROBOT IRP6_MECHATRONIKA
	if (config.return_int_value("is_irp6_mechatronika_active", "[ui]")) {
		created_robot = new robot::irp6_mechatronika (*this);
		robot_m[lib::ROBOT_IRP6_MECHATRONIKA] = created_robot;
	}

	// ROBOT POLYCRANK
	if (config.return_int_value("is_polycrank_active", "[ui]")) {
		created_robot = new robot::polycrank (*this);
		robot_m[lib::ROBOT_POLYCRANK] = created_robot;
	}

	// ROBOT_ELECTRON
	if (config.return_int_value("is_electron_robot_active", "[ui]")) {
		created_robot = new robot::robot (lib::ROBOT_ELECTRON, "[ecp_electron]", *this);
		robot_m[lib::ROBOT_ELECTRON] = created_robot;
	}

	// ROBOT_SPEECHRECOGNITION
	if (config.return_int_value("is_speechrecognition_active", "[ui]")) {
		created_robot = new robot::robot (lib::ROBOT_SPEECHRECOGNITION, "[ecp_speechrecognition]", *this);
		robot_m[lib::ROBOT_SPEECHRECOGNITION] = created_robot;
	}

	// ROBOT_FESTIVAL
	if (config.return_int_value("is_festival_active", "[ui]")) {
		created_robot = new robot::robot (lib::ROBOT_FESTIVAL, "[ecp_festival]", *this);
		robot_m[lib::ROBOT_FESTIVAL] = created_robot;
	}
}


// methods for mp template to redefine in concrete classes
void task::task_initialization(void)
{
	sr_ecp_msg->message("MP loaded");
}

void task::main_task_algorithm(void)
{
}

// metody do obslugi najczesniej uzywanych generatorow
void task::set_next_playerpos_goal (lib::ROBOT_ENUM robot_l, const lib::playerpos_goal_t &goal)
{
	// setting the next ecps state
	generator::set_next_ecps_state mp_snes_gen(*this);

	mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];

	mp_snes_gen.configure(goal);

	mp_snes_gen.Move();
}

// metody do obslugi najczesniej uzywanych generatorow
void task::set_next_ecps_state (int l_state, int l_variant, const char* l_string, int number_of_robots, ... )
{
	// setting the next ecps state
	generator::set_next_ecps_state mp_snes_gen (*this);

	va_list arguments;                     // A place to store the list of arguments
	lib::ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (lib::ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_snes_gen.configure (l_state, l_variant, l_string);

	mp_snes_gen.Move();
}

// delay MP replacement
void task::wait_ms (int _ms_delay) // zamiast delay
{
	generator::delay_ms_condition mp_ds_ms (*this, _ms_delay);

	mp_ds_ms.Move();
}

// send_end_motion
void task::send_end_motion_to_ecps (int number_of_robots, ... )
{
	generator::send_end_motion_to_ecps mp_semte_gen (*this);

	va_list arguments;                     // A place to store the list of arguments
	lib::ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (lib::ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_semte_gen.Move();
}

// send_end_motion
void task::send_end_motion_to_ecps (int number_of_robots, lib::ROBOT_ENUM *properRobotsSet)
{
	generator::send_end_motion_to_ecps mp_semte_gen (*this);

	lib::ROBOT_ENUM robot_l;

	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = properRobotsSet[x]; // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}

	mp_semte_gen.Move();
}

void task::run_extended_empty_gen (bool activate_trigger, int number_of_robots, ... )
{
	generator::extended_empty mp_ext_empty_gen (*this);

	va_list arguments;                     // A place to store the list of arguments
	lib::ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (lib::ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_ext_empty_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_ext_empty_gen.configure (activate_trigger);

	mp_ext_empty_gen.Move();
}

void task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... )
{
	// CZYNNOSCI WSTEPNE
	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	map <lib::ROBOT_ENUM, robot::robot*> robots_to_move, robots_to_wait_for_task_termination;
	map <lib::ROBOT_ENUM, robot::robot*> robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;
	map <lib::ROBOT_ENUM, robot::robot*>::iterator robots_map_iter;

	// powolanie generatora i jego konfiguracja
	generator::extended_empty mp_ext_empty_gen (*this);
	mp_ext_empty_gen.configure (false);

	// na podstawie argumentow wywolania biezacej metody
	va_list arguments;    // A place to store the list of arguments
	lib::ROBOT_ENUM robot_l;

	// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
	va_start ( arguments, number_of_robots_to_wait_for_task_termin);
	// najpierw zbior robots_to_move...
	for ( int x = 0; x < number_of_robots_to_move; x++ )        // Loop until all numbers are added
	{
		robot_l = (lib::ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.

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
		robot_l = (lib::ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		if (robot_m.count(robot_l) == 0)
		{
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
		} else {
			robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
		}
	}
	va_end ( arguments );              // Cleans up the list

	// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

	for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination.begin();
	robot_m_iterator != robots_to_wait_for_task_termination.end(); robot_m_iterator++) {

		robots_map_iter = robots_to_move.find(robot_m_iterator->first);
		if (robots_map_iter == robots_to_move.end()) {
			sr_ecp_msg->message (lib::SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
			throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
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
		for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robots_to_move_tmp.begin();
		robot_m_iterator != robots_to_move_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == lib::TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
				robots_to_move.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie zbioru robots_to_wait_for_task_termination
		for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination_tmp.begin();
		robot_m_iterator != robots_to_wait_for_task_termination_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == lib::TASK_TERMINATED  ) {
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

void task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, lib::ROBOT_ENUM *robotsToMove, lib::ROBOT_ENUM *robotsWaitingForTaskTermination)
{
	// CZYNNOSCI WSTEPNE
	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	map <lib::ROBOT_ENUM, robot::robot*> robots_to_move, robots_to_wait_for_task_termination;
	map <lib::ROBOT_ENUM, robot::robot*> robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;
	map <lib::ROBOT_ENUM, robot::robot*>::iterator robots_map_iter;

	// powolanie generatora i jego konfiguracja
	generator::extended_empty mp_ext_empty_gen (*this);
	mp_ext_empty_gen.configure (false);

	// na podstawie argumentow wywolania biezacej metody
	//va_list arguments;    // A place to store the list of arguments
	lib::ROBOT_ENUM robot_l;

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

	for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination.begin();
	robot_m_iterator != robots_to_wait_for_task_termination.end(); robot_m_iterator++) {

		robots_map_iter = robots_to_move.find(robot_m_iterator->first);
		if (robots_map_iter == robots_to_move.end()) {
			sr_ecp_msg->message (lib::SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
			throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
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
		for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robots_to_move_tmp.begin();
		robot_m_iterator != robots_to_move_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == lib::TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
				robots_to_move.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie zbioru robots_to_wait_for_task_termination
		for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination_tmp.begin();
		robot_m_iterator != robots_to_wait_for_task_termination_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == lib::TASK_TERMINATED  ) {
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

int task::mp_receive_pulse (common::mp_receive_pulse_struct_t* outputs, MP_RECEIVE_PULSE_MODE tryb)
{

	bool exit_loop = false;

	struct sigevent event;
	event.sigev_notify = SIGEV_UNBLOCK;

	while (!exit_loop) {

		if (tryb == WITH_TIMEOUT) {
			TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE,  &event, NULL, NULL );// by Y zamiast creceive
		}

		outputs->rcvid = MsgReceive_r (mp_pulse_attach->chid, &(outputs->pulse_msg), sizeof(_pulse_msg), &(outputs->msg_info));

		if (outputs->rcvid < 0) {/* Error condition, exit */

			outputs->e = errno;
			exit_loop = true;
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

			exit_loop = true;
			continue;
		}

		if (outputs->rcvid > 0) {
			/* A QNX IO message received, reject */
			// ECP lub UI wywolalo name_open
			if (outputs->pulse_msg.hdr.type >= _IO_BASE && outputs->pulse_msg.hdr.type <= _IO_MAX) {
				// 	  printf("w MP_TRIGGER _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
				//  MsgError(rcvid, ENOSYS);
				//			  printf("mp_receive_ecp_pulse_return_t name_open: %d, %d\n", info.pid, info.scoid);
				MsgReply (outputs->rcvid, EOK, 0, 0);
				exit_loop = true;
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

int task::check_and_optional_wait_for_new_pulse (common::mp_receive_pulse_struct_t* outputs,
		WAIT_FOR_NEW_PULSE_ENUM process_mode, MP_RECEIVE_PULSE_MODE desired_wait_mode)
{

	int ret;
	bool exit_from_while = false;
	bool desired_pulse_found = false;

	MP_RECEIVE_PULSE_MODE current_wait_mode = WITH_TIMEOUT;

	// checking of already registered pulses

	if ((process_mode == NEW_ECP_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
		for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robot_m.begin();
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

		if (ret < 0) {
			if (outputs->e != ETIMEDOUT) {
				// tu ma byc wyjatek
				fprintf (stderr, "MP: MsgReceive() na kanale ecp_pusle: %s @ %s:%d\n", strerror(ret), __FILE__, __LINE__);
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
			for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robot_m.begin();
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


int task::mp_wait_for_name_open(common::mp_receive_pulse_struct_t* outputs)
{
	int ret;
	bool wyjscie = false;

	while (!wyjscie) {
		ret = mp_receive_pulse (outputs, WITHOUT_TIMEOUT);
		// jakis inny robot wyslal puls
		if (ret < 0) {
			// TODO: tu ma byc wyjatek
			fprintf (stderr, "MP: MsgReceive() na kanale ecp_pusle: %s @ %s:%d\n", strerror(ret), __FILE__, __LINE__);
		} else if (ret == 0) {

			// wstawiamy informacje o pulsie ktory przyszedl od innego robota
			for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robot_m.begin();
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

// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE

void task::mp_receive_ui_or_ecp_pulse (map <lib::ROBOT_ENUM, robot::robot*>& _robot_m, generator::generator& the_generator )
{

	enum MP_STATE_ENUM
	{
		MP_STATE_RUNNING,
		MP_STATE_PAUSED
	};

	MP_STATE_ENUM mp_state = MP_STATE_RUNNING;
	int rcvid;
	common::mp_receive_pulse_struct_t input;

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
					sr_ecp_msg->message(lib::SYSTEM_ERROR, input.e, "MP:Creceive STOP pulse from UI failed");
					throw common::MP_main_error (lib::SYSTEM_ERROR, (uint64_t) 0);
				} else {
					ui_exit_from_while = true;
					continue;
				}
			} else if (mp_state == MP_STATE_PAUSED) {
				perror("Creceive RESUME proxy from UI failed ?\n");
				sr_ecp_msg->message(lib::SYSTEM_ERROR, input.e, "MP: receive RESUME pulse from UI failed");
				throw common::MP_main_error (lib::SYSTEM_ERROR, (uint64_t) 0);
			}
		} else if (rcvid == 0) {
			if (ui_new_pulse) {

				ui_new_pulse = false;

				if (ui_pulse_code == MP_STOP) {
					terminate_all (_robot_m);
					throw common::MP_main_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
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
				for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = robot_m.begin();
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

void task::initialize_communication()
{
	std::string sr_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	std::string mp_attach_point =	config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_attach_point");

	if (( sr_ecp_msg = new lib::sr_ecp(lib::MP, mp_attach_point, sr_net_attach_point)) == NULL) { // Obiekt do komuniacji z SR
		perror ( "Unable to locate SR\n");

		throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	// Rejestracja procesu MP
#if !defined(USE_MESSIP_SRR)
	if ((mp_attach = name_attach(NULL, mp_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
#else
		if ((mp_attach = messip_channel_create(NULL, mp_attach_point.c_str(), MESSIP_NOTIMEOUT, 0)) == NULL) {
#endif
			uint64_t e = errno; // kod bledu systemowego
			perror("Failed to attach Master Process\n");
			sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "MP: Failed to name attach");

			throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
		}

		std::string mp_pulse_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_pulse_attach_point");

		// Rejestracja kanalu dla pulsow z procesu UI i ECP
#if !defined(USE_MESSIP_SRR)
		if ((mp_pulse_attach = name_attach(NULL, mp_pulse_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
#else
			if ((mp_pulse_attach = messip_channel_create(NULL, mp_pulse_attach_point.c_str(), MESSIP_NOTIMEOUT, 0)) == NULL) {
#endif
				uint64_t e = errno; // kod bledu systemowego
				perror("Failed to attach UI Pulse chanel for Master Process\n");
				sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "MP: Failed to name attach  UI Pulse");


				throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			}

			common::mp_receive_pulse_struct_t outputs;
			if (mp_wait_for_name_open(&outputs) > 0) {
				ui_scoid = outputs.msg_info.scoid;
			}
		}
		// -------------------------------------------------------------------

		void task::wait_for_start ()
		{
			// Oczekiwanie na zlecenie START od UI

			while (1) {
				common::mp_receive_pulse_struct_t input;
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


		void task::wait_for_stop (common::WAIT_FOR_STOP_ENUM tryb)
		{
			// Oczekiwanie na zlecenie STOP od UI

			sr_ecp_msg->message("To terminate user program click STOP icon");
			bool wyjscie = false;

			while (!wyjscie) {
				common::mp_receive_pulse_struct_t input;
				int rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);

				if (rcvid == -1)/* Error condition, exit */
				{
					if (input.e != ETIMEDOUT)
					{
						perror("Receive StopProxy failed (MP)");
						sr_ecp_msg->message(lib::SYSTEM_ERROR, input.e, "MP: Receive StopProxy failed");
						switch (tryb) {
						case common::MP_THROW:
							throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
							break;
						case common::MP_EXIT:
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

		void task::start_all (map <lib::ROBOT_ENUM, robot::robot*>& _robot_m)
		{
			// Wystartowanie wszystkich ECP

			map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator;
			map <lib::ROBOT_ENUM, robot::robot*> robots_m_tmp, robots_m_tmp2;

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
							throw common::MP_main_error(lib::NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_START_ALL);
						}
					} else {
						// dodaj robota do listy jeszcze nie obsluzonych
						robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
					}
				}

				// ponowne przepisanie map
				robots_m_tmp = robots_m_tmp2;

				if (!(robots_m_tmp.empty())) {
					common::mp_receive_pulse_struct_t input;
					check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
				}
			}
		}
		// ------------------------------------------------------------------------


		// ------------------------------------------------------------------------
		void task::execute_all (map <lib::ROBOT_ENUM, robot::robot*>& _robot_m)
		{
			// Wystartowanie wszystkich ECP
			// do przepisania wg http://www.thescripts.com/forum/thread62378.html

			map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator;
			map <lib::ROBOT_ENUM, robot::robot*> robots_m_tmp, robots_m_tmp2;

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
							throw common::MP_main_error(lib::NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL);
						}
					} else {
						// dodaj robota do listy jeszcze nie obsluzonych
						robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
					}
				}

				// ponowne przepisanie map
				robots_m_tmp = robots_m_tmp2;

				if (!(robots_m_tmp.empty())) {
					common::mp_receive_pulse_struct_t input;
					check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
				}
			}
		}
		// ------------------------------------------------------------------------


		// ------------------------------------------------------------------------
		void task::terminate_all (map <lib::ROBOT_ENUM, robot::robot*>& _robot_m)
		{
			// Zatrzymanie wszystkich ECP
			map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator;

			map <lib::ROBOT_ENUM, robot::robot*> robots_m_tmp, robots_m_tmp2;

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
							throw common::MP_main_error(lib::NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL);
						}
					} else {
						// dodaj robota do listy jeszcze nie obsluzonych
						robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
					}
				}

				// ponowne przepisanie map
				robots_m_tmp = robots_m_tmp2;

				if (!(robots_m_tmp.empty())) {
					common::mp_receive_pulse_struct_t input;
					check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
				}
			}
		}
		// ------------------------------------------------------------------------


		// ------------------------------------------------------------------------
		void task::kill_all_ECP (map <lib::ROBOT_ENUM, robot::robot*>& _robot_m)
		{
			// Zabicie wszystkich ECP z mapy
			for (map <lib::ROBOT_ENUM, robot::robot*>::iterator robot_m_iterator = _robot_m.begin();
			robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
#if defined(PROCESS_SPAWN_RSH)
				kill(robot_m_iterator->second->ECP_pid, SIGTERM);
#else
				SignalKill(robot_m_iterator->second->nd, robot_m_iterator->second->ECP_pid, 0, SIGTERM, 0, 0);
#endif
			}
		}
		// ------------------------------------------------------------------------


	} // namespace task
} // namespace mp
} // namespace mrrocpp
