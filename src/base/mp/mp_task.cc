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

#include "mp_exceptions.h"
#include "mp_task.h"
#include "generator/mp_g_wait_for_task_termination.h"
#include "generator/mp_g_delay_ms_condition.h"
#include "generator/mp_g_set_next_ecps_state.h"
#include "generator/mp_g_send_end_motion_to_ecps.h"
#include "mp_robot.h"

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
	} catch (exception::nfe & e) {
		exit(EXIT_FAILURE);
	}
	terminate_all();
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
void task::set_next_ecp_state(const std::string & l_state, int l_variant, const char* l_string, int str_len, const lib::robot_name_t & robot_name)
{
	// setting the next ecps state
	generator::set_next_ecps_state mp_snes_gen(*this);

	// Copy given robots to the map container
	mp_snes_gen.robot_m[robot_name] = robot_m[robot_name];

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

	va_start(arguments, number_of_robots);
	// Initializing arguments to store all values after num

	// Copy given robots to the map container
	va_to_robot_map(number_of_robots, arguments, robot_m, mp_semte_gen.robot_m);

	va_end(arguments);
	// Cleans up the list

	mp_semte_gen.Move();
}

// send_end_motion
void task::wait_for_task_termination(bool activate_trigger, int number_of_robots, ...)
{
	generator::wait_for_task_termination wtf_gen(*this);

	va_list arguments; // A place to store the list of arguments

	va_start(arguments, number_of_robots);
	// Initializing arguments to store all values after num

	// Copy given robots to the map container
	va_to_robot_map(number_of_robots, arguments, robot_m, wtf_gen.robot_m);

	va_end(arguments);
	// Cleans up the list

	wtf_gen.configure(activate_trigger);

	wtf_gen.Move();
}

void task::wait_for_task_termination(bool activate_trigger, int number_of_robots, const std::vector <lib::robot_name_t> & robotSet)
{
	generator::wait_for_task_termination wtf_gen(*this);

	BOOST_FOREACH(lib::robot_name_t robotName, robotSet)
			{
				wtf_gen.robot_m[robotName] = robot_m[robotName];
			}

	wtf_gen.configure(activate_trigger);

	wtf_gen.Move();
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

//
// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
//
// intended use:
//
// when the_generator.wait_for_ECP_pulse is set
//     1) block for ECP pulse and react to UI pulses
// otherwise
//     2) peak for UI pulse and eventually react for in in pause/resume/stop/trigger cycle
void task::receive_ui_or_ecp_message(generator::generator & the_generator)
{

	// najpierw kasujemy znacznik swiezosci buforow
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				if (robot_node.second->reply.isFresh()) {

					robot_node.second->reply.markAsUsed();

				}
			}

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
		//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message loop");
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
						terminate_all();
						BOOST_THROW_EXCEPTION(exception::nfe() << lib::exception::mrrocpp_error0(ECP_STOP_ACCEPTED));
					case MP_PAUSE:

						mp_state = MP_PAUSED;
						pause_all();
						ui_exit_from_while = false;
						continue;
					default:
						break;
				}

				if (mp_state == MP_PAUSED) { // oczekujemy na resume
					if (ui_pulse.Get() == MP_RESUME) { // odebrano resume
						mp_state = MP_RUNNING;
						resume_all();
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
				//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse the_generator.wait_for_ECP_pulse");
				BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
						{
							if (robot_node.second->reply.isFresh()) {
								//					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse received");

								//	 if (debug_tmp)	printf("wait_for_ECP_pulse r: %d, pc: %d\n", robot_node.first, robot_node.second->ui_pulse_code);
								ecp_exit_from_while = true;

								robot_node.second->ecp_errors_handler();

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
	//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message end");

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
	//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_stop koniec");

}

void task::wait_for_all_robots_acknowledge()
{
	// Container for awaiting acknowledgements
	common::robots_t not_confirmed = robot_m;

	//	BOOST_FOREACH(const common::robot_pair_t & robot_node, not_confirmed)
	//	{
	//		robot_node.second->ecp_reply_package.reply = lib::INCORRECT_MP_COMMAND;
	//	}

	// Wait for ACK from all the robots
	while (!not_confirmed.empty()) {
		//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_all_robots_acknowledge przed receive");
		ReceiveSingleMessage(true);
		//		sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_all_robots_acknowledge za receive");
		BOOST_FOREACH(const common::robot_pair_t & robot_node, not_confirmed)
				{
					if (robot_node.second->reply.isFresh()
							&& robot_node.second->reply.Get().reply == lib::ECP_ACKNOWLEDGE) {
						robot_node.second->reply.markAsUsed();
						not_confirmed.erase(robot_node.first);

					}
				}
	}
}

void task::start_all()
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->start_ecp();

			}

	wait_for_all_robots_acknowledge();
}

void task::pause_all()
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->pause_ecp();
			}

}

void task::resume_all()
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->resume_ecp();
			}

}

void task::terminate_all()
{
	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "terminate_all poczatek");
	// Zatrzymanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				//if(robot_node.second->reply.Get().reply != lib::TASK_TERMINATED)
				robot_node.second->terminate_ecp();
			}

	wait_for_all_robots_acknowledge();
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
