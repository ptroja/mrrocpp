/*!
 * @file
 * @brief File contains mp base task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstdlib>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include "base/lib/datastr.h"

#include "mp_exceptions.h"
#include "mp_task_base.h"
#include "generator/mp_g_wait_for_task_termination.h"
#include "generator/mp_g_send_end_motion_to_ecps.h"
#include "mp_robot.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace mp {
namespace task {

task_base::task_base(lib::configurator &_config) :
		ecp_mp::task::task(_config), ui_pulse(*this, "MP_PULSE")
{
	// initialize communication with other processes
	initialize_communication();
}

task_base::~task_base()
{
	// Remove (kill) all ECP from the container
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				delete robot_node.second;
			}
}

void task_base::stop_and_terminate()
{
	sr_ecp_msg->message("To terminate MP click STOP icon");
	try {
		wait_for_stop();
	} catch (exception::nfe & e) {
		exit(EXIT_FAILURE);
	}
	terminate_all();
}

//
// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
//
// intended use:
//
// when the_generator.wait_for_ECP_message is set
//     1) block for ECP message and react to UI pulses
// otherwise
//     2) peak for UI pulse and eventually react for in in pause/resume/stop/trigger cycle
void task_base::receive_ui_or_ecp_message(generator::generator & the_generator)
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
	bool ecp_exit_from_while = (the_generator.wait_for_ECP_message) ? false : true;

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
				//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse ui_pulse.isFresh()");

				ui_pulse.markAsUsed();

				switch (ui_pulse.Get())
				{
					case MP_STOP:
						terminate_all();
						BOOST_THROW_EXCEPTION(exception::nfe() << lib::exception::mrrocpp_error0(ECP_STOP_ACCEPTED));
						break;
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
						the_generator.set_trigger();
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

			if (the_generator.wait_for_ECP_message) {
				//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse the_generator.wait_for_ECP_message");
				BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
						{
							if (robot_node.second->reply.isFresh()) {
								//					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "receive_ui_or_ecp_message pulse received");

								//	 if (debug_tmp)	printf("wait_for_ECP_message r: %d, pc: %d\n", robot_node.first, robot_node.second->ui_pulse_code);
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
void task_base::initialize_communication()
{
	const std::string sr_net_attach_point = config.get_sr_attach_point();

	// Obiekt do komuniacji z SR
	sr_ecp_msg = (boost::shared_ptr <lib::sr_ecp>) new lib::sr_ecp(lib::MP, "mp", sr_net_attach_point); // Obiekt do komuniacji z SR
}

void task_base::wait_for_start()
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

void task_base::wait_for_stop(void)
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

void task_base::wait_for_all_robots_acknowledge()
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

void task_base::start_all()
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->start_ecp();

			}

	wait_for_all_robots_acknowledge();
}

void task_base::pause_all()
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->pause_ecp();
			}

}

void task_base::resume_all()
{
	// Wystartowanie wszystkich ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->resume_ecp();
			}

}

void task_base::terminate_all()
{
	//sr_ecp_msg->message(lib::NON_FATAL_ERROR, "terminate_all poczatek");
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
