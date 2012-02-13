/*!
 * @file
 * @brief File contains ecp base task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cstring>
#include <unistd.h>
#include <cerrno>
#include <cctype>
#include <cstdio>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "ecp_task.h"
#include "ecp_robot.h"
#include "ecp_exceptions.h"
#include "ecp_generator.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

task_base::task_base(lib::configurator &_config, boost::shared_ptr <robot::ecp_robot_base> & robot_ref) :
		ecp_mp::task::task(_config),
		ecp_m_robot(robot_ref),
		MP(lib::MP_SECTION),
		reply(MP, _config.section_name),
		command(*this, "MP_COMMAND"),
		mp_command(command.access),
		mp_2_ecp_next_state_string(mp_command.ecp_next_state.next_state),
		continuous_coordination(false),
		mp_2_ecp_next_state_string_handler_active(true)
{
	initialize_communication();
}

void task_base::register_generator(generator::generator_base* _gen)
{
	const std::string & gen_name = _gen->generator_name;

	if (gen_name != EMPTY_SUBTASK_GENERATOR_NAME) {
		if (generator_m.find(gen_name) == generator_m.end()) {
			generator_m[gen_name] = _gen;
		} else {
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			ss << "Generator name already registered: " << gen_name;
			sr_ecp_msg->message(lib::FATAL_ERROR, ss.str().c_str());
		}
	} else {
		sr_ecp_msg->message(lib::FATAL_ERROR, "No name specified for generator");
	}
}

void task_base::subtasks_and_generators_dispatcher()
{

	if (generator_m.find(mp_2_ecp_next_state_string) != generator_m.end()) {
		//sr_ecp_msg->message(lib::NON_FATAL_ERROR, mp_2_ecp_next_state_string);
		generator_m.at(mp_2_ecp_next_state_string)->conditional_execution();
	} else {
		if (!mp_2_ecp_next_state_string_handler_active) {
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			ss << "ecp dispatcher failure (label not recognized): " << mp_2_ecp_next_state_string;
			sr_ecp_msg->message(lib::FATAL_ERROR, ss.str().c_str());
		}
	}

}

void task_base::main_task_algorithm(void)
{
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		// message with current generator being executed
		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "Order received: " << mp_2_ecp_next_state_string;
		sr_ecp_msg->message(ss.str().c_str());

		mp_2_ecp_next_state_string_handler();

		subtasks_and_generators_dispatcher();

		termination_notice();
	} //end for
}

void task_base::mp_2_ecp_next_state_string_handler(void)
{
	mp_2_ecp_next_state_string_handler_active = false;
}

void task_base::ecp_stop_accepted_handler(void)
{
}

task_base::~task_base()
{
	// TODO: error check
	messip::port_delete(trigger_attach);
}

bool task_base::pulse_check()
{
	int32_t type, subtype;
	int recvid;
	if ((recvid = messip::port_receive_pulse(trigger_attach, type, subtype, 0)) == -1) {
		perror("messip::port_receive()");
		return false;
	}

	if (recvid == MESSIP_MSG_NOREPLY && type == ECP_TRIGGER) {
		return true;
	}

	return false;
}

// ---------------------------------------------------------------
void task_base::initialize_communication()
{
	const std::string ecp_attach_point = config.get_ecp_attach_point();

	const std::string sr_net_attach_point = config.get_sr_attach_point();

	// Obiekt do komuniacji z SR
	sr_ecp_msg = (boost::shared_ptr <lib::sr_ecp>) new lib::sr_ecp(lib::ECP, ecp_attach_point, sr_net_attach_point);

	//	std::cout << "ecp: Opening MP pulses channel at '" << mp_pulse_attach_point << "'" << std::endl;

	const std::string trigger_attach_point = config.get_ecp_trigger_attach_point();

	if ((trigger_attach = messip::port_create(trigger_attach_point)) == NULL) {
		int e = errno; // kod bledu systemowego
		perror("Failed to attach TRIGGER pulse chanel for ecp");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "Failed  Failed to name attach (trigger pulse)");
		BOOST_THROW_EXCEPTION(exception::se());
	}
}
// -------------------------------------------------------------------

// Badanie typu polecenia z MP
lib::MP_COMMAND task_base::mp_command_type(void) const
{
	return mp_command.command;
}

// Ustawienie typu odpowiedzi z ECP do MP
void task_base::set_ecp_reply(lib::ECP_REPLY ecp_r)
{
	ecp_reply.reply = ecp_r;
}

// Informacja dla MP o zakonczeniu zadania uzytkownika
void task_base::termination_notice(void)
{
	if (mp_command_type() != lib::END_MOTION) {
		set_ecp_reply(lib::TASK_TERMINATED);
		reply.Send(ecp_reply);
	}
}

// Petla odbierania wiadomosci.
void task_base::wait_for_stop(void)
{
	while (command.Get().command != lib::STOP) {
		ReceiveSingleMessage(true);
	}
	set_ecp_reply(lib::ECP_ACKNOWLEDGE);

	// Reply with ACK
	reply.Send(ecp_reply);
}

// Oczekiwanie na polecenie START od MP
void task_base::wait_for_start(void)
{
	// Awaiting for the START command
	bool start_received = false;

	while (!start_received) {
		// wait for a new message
		ReceiveSingleMessage(true);

		// ignore non-MP commands
		if (!command.isFresh())
			continue;

		command.markAsUsed();

		switch (mp_command.command)
		{
			case lib::START_TASK:
				// by Y - ECP_ACKNOWLEDGE zamienione na lib::TASK_TERMINATED w celu uproszczenia oprogramowania zadan wielorobotowych
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);

				// Reply with ACK
				reply.Send(ecp_reply);

				// OK, ready to start!
				start_received = true;

				break;
			case lib::STOP:
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				// Reply with ACK
				reply.Send(ecp_reply);
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(ECP_STOP_ACCEPTED));
				break;
			default:
				set_ecp_reply(lib::INCORRECT_MP_COMMAND);
				// Reply with NACK
				reply.Send(ecp_reply);
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_MP_COMMAND));
				break;
		}
	}

	sr_ecp_msg->message("ecp user program is running");
}

// Oczekiwanie na kolejne zlecenie od MP
void task_base::get_next_state(void)
{
	bool next_state_received = false;

	while (!next_state_received) {
		while (!command.isFresh()) {
			ReceiveSingleMessage(true);
		}

		command.markAsUsed();

		switch (mp_command.command)
		{
			case lib::NEXT_STATE:
				// Reply with ACK
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				reply.Send(ecp_reply);

				next_state_received = true;
				break;
			case lib::PAUSE_TASK:
				wait_for_resume();
				break;
			case lib::STOP:
				// Reply with ACK
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				reply.Send(ecp_reply);
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(ECP_STOP_ACCEPTED));
				break;
			default:
				// Reply with NACK
				set_ecp_reply(lib::INCORRECT_MP_COMMAND);
				reply.Send(ecp_reply);
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_MP_COMMAND));
				break;
		}
	}
}

// Receive a message from MP
bool task_base::peek_mp_message()
{
	command.markAsUsed();

	const bool block = continuous_coordination;

	if (ReceiveSingleMessage(block)) {
		if (command.isFresh()) {

			switch (mp_command.command)
			{

				case lib::NEXT_POSE:

					if (continuous_coordination) {
						reply.Send(ecp_reply);
					} else {
						sr_ecp_msg->message(lib::NON_FATAL_ERROR, "STRANGE COMMAND (NEXT_POSE) for non continuous_coordination");
					}

					break;

				case lib::END_MOTION:
					command.markAsUsed();
					return true;

					break;

				case lib::STOP:
					set_ecp_reply(lib::ECP_ACKNOWLEDGE);
					//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "peek_mp_message lib::STOP");
					command.markAsUsed();
					// Reply with ACK
					reply.Send(ecp_reply);
					BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(ECP_STOP_ACCEPTED));
					break;
				case lib::PAUSE_TASK:
					//	set_ecp_reply(lib::ECP_ACKNOWLEDGE);
					//sr_ecp_msg->message(lib::NON_FATAL_ERROR, "peek_mp_message lib::PAUSE_TASK");
					command.markAsUsed();
					// Reply with ACK
					//reply.Send(ecp_reply);
					wait_for_resume();
					break;
				default:
					set_ecp_reply(lib::INCORRECT_MP_COMMAND);
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "peek_mp_message lib::INCORRECT_MP_COMMAND");
					command.markAsUsed();
					// Reply with NACK
					reply.Send(ecp_reply);
					BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_MP_COMMAND));
					break;
			}
		}

	}

	return false;
}

void task_base::wait_for_resume()
{
	// Awaiting for the RESUME command
	bool resume_received = false;

	while (!resume_received) {

		// wait for a new message
		ReceiveSingleMessage(true);

		// ignore non-MP messages
		if (!command.isFresh())
			continue;

		command.markAsUsed();

		switch (command.Get().command)
		{
			case lib::STOP:
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_resume lib::STOP");

				// Reply with ACK
				reply.Send(ecp_reply);
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(ECP_STOP_ACCEPTED));
				break;
			case lib::RESUME_TASK:
				//	set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				//	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_resume lib::RESUME_TASK");

				// Reply with ACK
				//	reply.Send(ecp_reply);

				// leave the receive loop
				resume_received = true;
				break;
			default:
				set_ecp_reply(lib::INCORRECT_MP_COMMAND);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_resume lib::INCORRECT_MP_COMMAND");

				// Reply with NACK
				reply.Send(ecp_reply);
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_MP_COMMAND));
				break;
		}
	}
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
