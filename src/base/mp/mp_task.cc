/*!
 * @file
 * @brief File contains mp base task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <csignal>
#include <cstdarg>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>
#include <fstream>
#include <cstring>

#include <boost/foreach.hpp>

#include "base/lib/datastr.h"

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_g_empty.h"
#include "base/mp/generator/mp_g_delay_ms_condition.h"
#include "base/mp/generator/mp_g_set_next_ecps_state.h"
#include "base/mp/generator/mp_g_send_end_motion_to_ecps.h"
#include "base/mp/generator/mp_g_extended_empty.h"
#include "base/mp/mp_robot.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace mp {
namespace task {

lib::fd_server_t task::mp_pulse_attach = lib::invalid_fd;

//! Utility function to put robot from va_list to STL map
static void va_to_robot_map(int num, va_list arguments, const common::robots_t & from, common::robots_t & to)
{
	for (int i = 0; i < num; ++i) // Loop until all numbers are added
	{
		lib::robot_name_t robot_l = (lib::robot_name_t) (va_arg ( arguments, const char* )); // Adds the next value in argument list to sum.
		if (from.count(robot_l) == 0) {
			std::cerr << "usunieto nadmiarowe roboty" << std::endl;
		} else {
			// find() is allowed on the 'const' reference and operator[] is not
			to[robot_l] = from.find(robot_l)->second;
		}
	}
}

// KONSTRUKTORY
task::task(lib::configurator &_config) :
	ecp_mp::task::task(_config), ui_pulse("MP_PULSE")
{
	// initialize communication with other processes
	initialize_communication();
}

task::~task()
{
	// Remove (kill) all ECP from the container
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					delete robot_node.second;
				}

	// TODO: check for error
	if (mp_pulse_attach != lib::invalid_fd) {

		messip::port_delete(mp_pulse_attach);

		mp_pulse_attach = lib::invalid_fd;
	}
}

void task::stop_and_terminate()
{
	sr_ecp_msg->message("To terminate MP click STOP icon");
	try {
		wait_for_stop();
	} catch (common::MP_main_error & e) {
		exit(EXIT_FAILURE);
	}
	terminate_all(robot_m);
}

// metody do obslugi najczesniej uzywanych generatorow
void task::set_next_playerpos_goal(lib::robot_name_t robot_l, const lib::playerpos_goal_t &goal)
{
	// setting the next ecps state
	generator::set_next_ecps_state mp_snes_gen(*this);

	mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];

	mp_snes_gen.configure(goal);

	mp_snes_gen.Move();
}

// metody do obslugi najczesniej uzywanych generatorow
void task::set_next_ecps_state(const std::string & l_state, int l_variant, const char* l_string, int str_len, int number_of_robots, ...)
{
	// setting the next ecps state
	generator::set_next_ecps_state mp_snes_gen(*this);

	va_list arguments; // A place to store the list of arguments

	va_start(arguments, number_of_robots); // Initializing arguments to store all values after num

	// Copy given robots to the map container
	va_to_robot_map(number_of_robots, arguments, robot_m, mp_snes_gen.robot_m);

	va_end(arguments); // Cleans up the list

	mp_snes_gen.configure(l_state, l_variant, l_string, str_len);
	mp_snes_gen.Move();
}

// delay MP replacement
void task::wait_ms(int _ms_delay) // zamiast delay
{
	generator::delay_ms_condition mp_ds_ms(*this, _ms_delay);

	mp_ds_ms.Move();
}

// send_end_motion
void task::send_end_motion_to_ecps(int number_of_robots, ...)
{
	generator::send_end_motion_to_ecps mp_semte_gen(*this);

	va_list arguments; // A place to store the list of arguments

	va_start(arguments, number_of_robots); // Initializing arguments to store all values after num

	// Copy given robots to the map container
	va_to_robot_map(number_of_robots, arguments, robot_m, mp_semte_gen.robot_m);

	va_end(arguments); // Cleans up the list

	mp_semte_gen.Move();
}

// send_end_motion
void task::send_end_motion_to_ecps(int number_of_robots, lib::robot_name_t *properRobotsSet)
{
	generator::send_end_motion_to_ecps mp_semte_gen(*this);

	for (int x = 0; x < number_of_robots; x++) // Loop until all numbers are added
	{
		lib::robot_name_t robot_l = properRobotsSet[x]; // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}

	mp_semte_gen.Move();
}

void task::run_extended_empty_gen_base(bool activate_trigger, int number_of_robots, ...)
{
	generator::extended_empty mp_ext_empty_gen(*this);

	va_list arguments; // A place to store the list of arguments

	va_start(arguments, number_of_robots); // Initializing arguments to store all values after num

	// Copy given robots to the map container
	va_to_robot_map(number_of_robots, arguments, robot_m, mp_ext_empty_gen.robot_m);

	va_end(arguments); // Cleans up the list

	mp_ext_empty_gen.configure(activate_trigger);

	mp_ext_empty_gen.Move();
}

void task::run_extended_empty_gen_and_wait(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ...)
{
	// TRANSLATE CALL ARGUMENTS

	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	common::robots_t robots_to_move, robots_to_wait_for_task_termination;

	// na podstawie argumentow wywolania biezacej metody
	va_list arguments; // A place to store the list of arguments

	// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
	va_start(arguments, number_of_robots_to_wait_for_task_termin);

	// najpierw zbior robots_to_move...
	va_to_robot_map(number_of_robots_to_move, arguments, robot_m, robots_to_move);

	// ...potem zbior robots_to_wait_for_task_termination
	va_to_robot_map(number_of_robots_to_wait_for_task_termin, arguments, robot_m, robots_to_wait_for_task_termination);

	va_end(arguments); // Cleans up the list

	// CALL THE PROPER METHOD
	run_extended_empty_gen_and_wait(robots_to_move, robots_to_wait_for_task_termination);
}

void task::run_extended_empty_gen_and_wait(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, lib::robot_name_t *robotsToMove, lib::robot_name_t *robotsWaitingForTaskTermination)
{
	// TRANSLATE CALL ARGUMENTS

	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	common::robots_t robots_to_move, robots_to_wait_for_task_termination;

	// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie

	// najpierw zbior robots_to_move...
	for (int x = 0; x < number_of_robots_to_move; x++) // Loop until all numbers are added
	{
		lib::robot_name_t robot_l = robotsToMove[x]; // Adds the next value in argument list to sum.

		if (robot_m.count(robot_l) == 0) {
			sr_ecp_msg->message("run_..._for_set_of_robots_... usunieto nadmiarowe roboty");
		} else {
			robots_to_move[robot_l] = robot_m[robot_l];
		}
	}
	// ...potem zbior robots_to_wait_for_task_termination
	for (int x = 0; x < number_of_robots_to_wait_for_task_termin; x++) // Loop until all numbers are added
	{
		lib::robot_name_t robot_l = robotsWaitingForTaskTermination[x]; // Adds the next value in argument list to sum.

		if (robot_m.count(robot_l) == 0) {
			sr_ecp_msg->message("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
		} else {
			robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
		}
	}

	// CALL THE PROPER METHOD
	run_extended_empty_gen_and_wait(robots_to_move, robots_to_wait_for_task_termination);
}

void task::run_extended_empty_gen_and_wait(common::robots_t & robots_to_move, common::robots_t & robots_to_wait_for_task_termination)
{
	// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination)
				{
					if (robots_to_move.count(robot_node.first) == 0) {
						sr_ecp_msg->message(lib::SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
						throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
					}
				}

	// GLOWNA PETLA

	do {
		sr_ecp_msg->message(lib::NON_FATAL_ERROR, "run_extended_empty_gen_and_wait do iteration");
		// aktualizacja ziorow robotow i sprawdzenie czy zbior robots_to_wait_for_task_termination nie jest juz pusty
		// wtedy wyjscie z petli

		// przygotowanie zapasowych list robotow
		bool all_robots_terminated = true;

		// sprawdzenie zbioru robots_to_wait_for_task_termination
		BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination)
					{
						if (robot_node.second->ecp_reply_package.reply != lib::TASK_TERMINATED) {
							all_robots_terminated = false;
						}
					}

		// sprawdzenie czy wszystkie roboty są w stanie TASK_TERMINATED
		// Jesli tak => wyjscie z petli i w konsekwencji wyjscie z calej metody
		if (all_robots_terminated) {
			sr_ecp_msg->message(lib::NON_FATAL_ERROR, "run_extended_empty_gen_and_wait break");
			break;
		}

		// przygotowanie zapasowych list robotow
		common::robots_t robots_to_move_tmp = robots_to_move;

		// sprawdzenie zbioru robots_to_move
		BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_move_tmp)
					{
						if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED) {
							robots_to_move.erase(robot_node.first);

							/* DEBUG START*/
							std::stringstream temp_message;
							temp_message << "TASK_TERMINATED robot (" << robot_node.second->robot_name << ")"
									<< std::endl;
							sr_ecp_msg->message(lib::NON_FATAL_ERROR, temp_message.str());
							/* DEBUG END*/
						}
					}

		// powolanie generatora i jego konfiguracja
		generator::extended_empty mp_ext_empty_gen(*this);
		mp_ext_empty_gen.configure(false);

		// przypisanie generatorowi mp_ext_empty_gen zbioru robots_to_move
		mp_ext_empty_gen.robot_m = robots_to_move;

		// uruchomienie generatora
		mp_ext_empty_gen.Move();

	} while (true);

}

//
// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
//
// intended use:
//
// when the_generator.wait_for_ECP_pulse is set
//     1) block for ECP pulse and react to UI pulses
// otherwise
//     2) peak for UI pulse and eventually react for in in pause/resume/stop/trigger cycle
void task::receive_ui_or_ecp_message(common::robots_t & _robot_m, generator::generator & the_generator)
{
	enum MP_STATE_ENUM
	{
		MP_RUNNING, MP_PAUSED
	} mp_state = MP_RUNNING;

	bool ui_exit_from_while = false;
	bool ecp_exit_from_while = (the_generator.wait_for_ECP_pulse) ? false : true;

	// 0 0 -> enter
	// 0 1 -> enter
	// 1 0 -> enter
	// 1 1 -> not enter
	while (!(ui_exit_from_while && ecp_exit_from_while)) {
		sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message loop");
		bool block;

		if (mp_state == MP_RUNNING) {
			// check for UI pulse or block for UI/ECP pulse
			block = ecp_exit_from_while ? false : true;
		} else {
			// block for UI resume/stop pulse
			block = true;
		}

		if (ReceiveSingleMessage(block)) {
			// UI Pulse arrived
			if (ui_pulse.isFresh()) {
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse ui_pulse.isFresh()");

				ui_pulse.markAsUsed();

				switch (ui_pulse.Get())
				{
					case MP_STOP:
						terminate_all(_robot_m);
						throw common::MP_main_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
					case MP_PAUSE:
						mp_state = MP_PAUSED;
						ui_exit_from_while = false;
						continue;
					default:
						break;
				}

				if (mp_state == MP_PAUSED) {// oczekujemy na resume
					if (ui_pulse.Get() == MP_RESUME) { // odebrano resume
						mp_state = MP_RUNNING;
						ui_exit_from_while = true;
					}
				} else {
					if (ui_pulse.Get() == MP_TRIGGER) { // odebrano trigger
						ui_exit_from_while = true;
						the_generator.trigger = true;
						// 2 ponizsze linie po dodaniu prawdziwej sporadycznej synchrnozniacji
						ui_exit_from_while = true;
						ecp_exit_from_while = true;
					}
				}
				continue;
			} else {
				if (mp_state == MP_RUNNING) {
					ui_exit_from_while = true;
				}
			}

			if (the_generator.wait_for_ECP_pulse) {
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse the_generator.wait_for_ECP_pulse");

				BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
							{
								if (robot_node.second->reply.isFresh()) {
									sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse received");

									robot_node.second->reply.markAsUsed();
									//	 if (debug_tmp)	printf("wait_for_ECP_pulse r: %d, pc: %d\n", robot_node.first, robot_node.second->ui_pulse_code);
									ecp_exit_from_while = true;
								}
							}
			} else {
				ecp_exit_from_while = true;
			}
		} else {
			if (mp_state == MP_RUNNING) {
				ui_exit_from_while = true;
			}
		}
	}
	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message end");

}

// -------------------------------------------------------------------
// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
// -------------------------------------------------------------------
void task::initialize_communication()
{
	const std::string sr_net_attach_point = config.get_sr_attach_point();

	// Obiekt do komuniacji z SR
	sr_ecp_msg = (boost::shared_ptr <lib::sr_ecp>) new lib::sr_ecp(lib::MP, "mp", sr_net_attach_point); // Obiekt do komuniacji z SR

	const std::string mp_pulse_attach_point = config.get_mp_pulse_attach_point();

	// Rejestracja kanalu dla pulsow z procesu UI
	registerBuffer(ui_pulse);
}

void task::wait_for_start()
{
	// Oczekiwanie na zlecenie START od UI

	while (ui_pulse.Get() != MP_START) {
		ReceiveSingleMessage(true);

		if (ui_pulse.isFresh()) {
			ui_pulse.markAsUsed();
		}

	}

	sr_ecp_msg->message("mp user program is running");
}

void task::wait_for_stop(void)
{
	// Oczekiwanie na zlecenie STOP od UI

	sr_ecp_msg->message("To terminate user program click STOP icon");

	while (ui_pulse.Get() != MP_STOP) {
		ReceiveSingleMessage(true);
		if (ui_pulse.isFresh()) {
			ui_pulse.markAsUsed();
		}
	}
	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_stop koniec");

}

void task::start_all(const common::robots_t & _robot_m)
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m)
				{
					robot_node.second->start_ecp();
				}

	// Container for awaiting acknowledgements
	common::robots_t not_confirmed = _robot_m;

	//	BOOST_FOREACH(const common::robot_pair_t & robot_node, not_confirmed)
	//	{
	//		robot_node.second->ecp_reply_package.reply = lib::INCORRECT_MP_COMMAND;
	//	}

	// Wait for ACK from all the robots
	while (!not_confirmed.empty()) {
		ReceiveSingleMessage(true);

		BOOST_FOREACH(const common::robot_pair_t & robot_node, not_confirmed)
					{
						if (robot_node.second->reply.isFresh() && robot_node.second->reply.Get().reply
								== lib::TASK_TERMINATED) {
							robot_node.second->reply.markAsUsed();
							not_confirmed.erase(robot_node.first);

						}
					}
	}
}

void task::execute_all(const common::robots_t & _robot_m)
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m)
				{
					if (robot_node.second->communicate_with_ecp) {
						if ((robot_node.second->mp_command.command == lib::STOP)
								|| (robot_node.second->mp_command.command == lib::END_MOTION)
								|| (robot_node.second->mp_command.command == lib::NEXT_STATE)
								|| (robot_node.second->continuous_coordination)) {
							robot_node.second->execute_motion();
						}
					}
				}
}

void task::terminate_all(const common::robots_t & _robot_m)
{
	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "terminate_all poczatek");
	// Zatrzymanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m)
				{
					//if(robot_node.second->reply.Get().reply != lib::TASK_TERMINATED)
					robot_node.second->terminate_ecp();
				}

	common::robots_t not_confirmed = _robot_m;

	// Wait for ACK from all the robots
	while (!not_confirmed.empty()) {
		sr_ecp_msg->message(lib::NON_FATAL_ERROR, "terminate_all not_confirmed.empty() poczatek");
		ReceiveSingleMessage(true);
		sr_ecp_msg->message(lib::NON_FATAL_ERROR, "terminate_all not_confirmed.empty() za receive ReceiveSingleMessage");

		BOOST_FOREACH(const common::robot_pair_t & robot_node, not_confirmed)
					{
						if (robot_node.second->reply.isFresh() && robot_node.second->reply.Get().reply
								== lib::ECP_ACKNOWLEDGE) {
							sr_ecp_msg->message(lib::NON_FATAL_ERROR, "terminate_all not_confirmed.empty() lib::TASK_TERMINATED");

							not_confirmed.erase(robot_node.first);
						}
					}
	}
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
