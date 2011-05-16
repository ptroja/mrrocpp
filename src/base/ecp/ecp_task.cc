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

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_sub_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ECP_main_error.h"
#include "base/ecp/ecp_generator.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

task_base::task_base(lib::configurator &_config) :
	ecp_mp::task::task(_config), continuous_coordination(false)
{
	initialize_communication();
}

void task_base::main_task_algorithm(void)
{
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		subtasks_conditional_execution();

		mp_2_ecp_next_state_string_handler();

		ecp_termination_notice();
	} //end for
}

void task_base::mp_2_ecp_next_state_string_handler(void)
{
}

void task_base::ecp_stop_accepted_handler(void)
{
}

task_base::~task_base()
{
	// TODO: error check

	messip::port_delete(trigger_attach);
	messip::port_delete(ecp_attach);
	messip::port_delete(MP_fd);

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
	std::string mp_pulse_attach_point = config.get_mp_pulse_attach_point();

	std::string ecp_attach_point = config.get_ecp_attach_point();
	std::string sr_net_attach_point = config.get_sr_attach_point();

	// Obiekt do komuniacji z SR
	sr_ecp_msg = new lib::sr_ecp(lib::ECP, config.robot_name, sr_net_attach_point);

	//	std::cout << "ecp: Opening MP pulses channel at '" << mp_pulse_attach_point << "'" << std::endl;


	if ((MP_fd = messip::port_connect(mp_pulse_attach_point)) == NULL)

	{
		int e = errno; // kod bledu systemowego
		fprintf(stderr, "ecp: Unable to locate MP_MASTER process at '%s'\n", mp_pulse_attach_point.c_str());
		perror("ecp: Unable to locate MP_MASTER process");
		throw ECP_main_error(lib::SYSTEM_ERROR, e);
	}

	// Rejestracja procesu ECP

	if ((ecp_attach = messip::port_create(ecp_attach_point)) == NULL) {
		int e = errno; // kod bledu systemowego
		perror("Failed to attach Effector Control Process");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	std::string trigger_attach_point = config.get_ecp_trigger_attach_point();

	if ((trigger_attach = messip::port_create(trigger_attach_point)) == NULL)

	{
		int e = errno; // kod bledu systemowego
		perror("Failed to attach TRIGGER pulse chanel for ecp");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "Failed  Failed to name attach (trigger pulse)");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
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
void task_base::ecp_termination_notice(void)
{
	if (mp_command_type() != lib::END_MOTION) {

		set_ecp_reply(lib::TASK_TERMINATED);
		mp_buffer_receive_and_send();
	}
}

// Wysyla puls do Mp przed oczekiwaniem na spotkanie
void task_base::send_pulse_to_mp(int pulse_code, int pulse_value)
{
	if (messip::port_send_pulse(MP_fd, pulse_code, pulse_value) < 0)

	{
		perror("MsgSendPulse()");
	}
}

void task_base::subtasks_conditional_execution()
{
	BOOST_FOREACH(const subtask_pair_t & subtask_node, subtask_m)
				{
					if (mp_2_ecp_next_state_string == subtask_node.first) {
						subtask_node.second->conditional_execution();
					}
				}
}

// Petla odbierania wiadomosci.
void task_base::ecp_wait_for_stop(void)
{
	// Wyslanie pulsu do MP

	bool mp_pulse_received = false;

	send_pulse_to_mp(ECP_WAIT_FOR_STOP);

	// Oczekiwanie na wiadomosc.
	int caller = -2;

	wait_for_randevous_with_mp(caller, mp_pulse_received);
	bool ecp_stop = false;
	if (mp_command_type() == lib::STOP) {
		set_ecp_reply(lib::ECP_ACKNOWLEDGE);
		ecp_stop = true;
	} else {
		set_ecp_reply(lib::ERROR_IN_ECP);
	}

	// Wyslanie odpowiedzi.
	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0) {
		uint64_t e = errno; // kod bledu systemowego
		perror("ecp: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Reply to MP failed");
		throw common::generator::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	// ew. odebranie pulsu z MP
	// sprawdzeniem czy MP wyslalo puls przed spotkaniem z ECP
	if ((!mp_pulse_received) && (mp_command.pulse_to_ecp_sent)) {
		caller = receive_mp_message(true);
	}

	if (!ecp_stop) {
		fprintf(stderr, "ecp_ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n", __FILE__, __LINE__);
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
}

// Oczekiwanie na polecenie START od MP
void task_base::ecp_wait_for_start(void)
{
	//std::cerr << "ecp ecp_wait_for_start 1" << std::endl;
	bool ecp_stop = false;
	bool mp_pulse_received = false;
	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_START);
	//	std::cerr << "ecp ecp_wait_for_start 2" << std::endl;
	int caller = -2;

	wait_for_randevous_with_mp(caller, mp_pulse_received);

	//	std::cerr << "ecp ecp_wait_for_start 3" << std::endl;

	switch (mp_command_type())
	{
		case lib::START_TASK:
			// by Y - ECP_ACKNOWLEDGE zamienione na lib::TASK_TERMINATED w celu uproszczenia oprogramowania zadan wielorobotowych
			set_ecp_reply(lib::TASK_TERMINATED);
			break;
		case lib::STOP:
			set_ecp_reply(lib::TASK_TERMINATED);
			ecp_stop = true;
			break;
		default:
			set_ecp_reply(lib::INCORRECT_MP_COMMAND);
			break;
	}

	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0) {
		uint64_t e = errno; // kod bledu systemowego
		perror("ecp: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Reply to MP failed");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	// ew. odebranie pulsu z MP
	// sprawdzeniem czy MP wyslalo puls przed spotkaniem z ECP
	if ((!mp_pulse_received) && (mp_command.pulse_to_ecp_sent)) {
		caller = receive_mp_message(true);
	}

	if (ecp_stop)
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == lib::INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n", __FILE__, __LINE__);
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	sr_ecp_msg->message("ecp user program is running");
}

// Oczekiwanie na kolejne zlecenie od MP
void task_base::get_next_state(void)
{

	bool mp_pulse_received = false;
	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_NEXT_STATE);

	int caller = -2;

	wait_for_randevous_with_mp(caller, mp_pulse_received);

	bool ecp_stop = false;

	switch (mp_command_type())
	{
		case lib::NEXT_STATE:
			set_ecp_reply(lib::ECP_ACKNOWLEDGE);
			break;
		case lib::STOP:
			set_ecp_reply(lib::ECP_ACKNOWLEDGE);
			ecp_stop = true;
			break;
		default:
			set_ecp_reply(lib::INCORRECT_MP_COMMAND);
			break;
	}

	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0) {
		uint64_t e = errno; // kod bledu systemowego
		perror("ecp: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Reply to MP failed");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	// ew. odebranie pulsu z MP
	// sprawdzeniem czy MP wyslalo puls przed spotkaniem z ECP
	if ((!mp_pulse_received) && (mp_command.pulse_to_ecp_sent)) {
		caller = receive_mp_message(true);
	}

	if (ecp_stop)
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == lib::INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d, mp_command_type() = %d\n", __FILE__, __LINE__, mp_command_type());
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	mp_2_ecp_next_state_string = mp_command.ecp_next_state.mp_2_ecp_next_state;
}

// Oczekiwanie na polecenie od MP
void task_base::wait_for_randevous_with_mp(int &caller, bool &mp_pulse_received)
{
	while (caller < 0) {

		caller = receive_mp_message(true);
		if (caller == MESSIP_MSG_NOREPLY) {
			mp_pulse_received = true;
			// przyszedl puls

		}
		//printf("mp_buffer_receive_and_send caller: %d\n", caller);
	}
}

// Oczekiwanie na polecenie od MP
bool task_base::mp_buffer_receive_and_send(void)
{
	//std::cerr << "ecp mp_buffer_receive_and_send 1" << std::endl;

	int caller = -2;

	bool mp_pulse_received = false;
	// ECP communication request
	if ((ecp_reply.reply == lib::TASK_TERMINATED) || (ecp_reply.reply == lib::ERROR_IN_ECP)
			|| (continuous_coordination)) {
		// wariant pierwszy ECP chce sie skomunikowac
		// Wyslanie pulsu do MP
		// zglaszamy chec i mozliwosc komunikacji
		send_pulse_to_mp(ECP_WAIT_FOR_COMMAND);

		wait_for_randevous_with_mp(caller, mp_pulse_received);

		return reply_to_mp(caller, mp_pulse_received);
		// MP communication request
	} else {
		// czy
		caller = receive_mp_message(false);

		if (caller == MESSIP_MSG_NOREPLY)

		{

			// przyszedl puls od mp, ktore chce sie komunikowac
			mp_pulse_received = true;
			send_pulse_to_mp(ECP_WAIT_FOR_COMMAND);
			caller = receive_mp_message(true);
			return reply_to_mp(caller, mp_pulse_received);
		}

	}

	//printf("mp_buffer_receive_and_send caller za: %d\n", caller);

	return true;
}

// Receive of mp message
bool task_base::reply_to_mp(int &caller, bool &mp_pulse_received)
{
	bool returned_value = true;
	bool ecp_stop = false;
	switch (mp_command_type())
	{
		case lib::NEXT_POSE:
			if ((ecp_reply.reply != lib::TASK_TERMINATED) && (ecp_reply.reply != lib::ERROR_IN_ECP))
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
			break;
		case lib::STOP:
			set_ecp_reply(lib::ECP_ACKNOWLEDGE);
			ecp_stop = true;
			break;
		case lib::END_MOTION:
			// dla ulatwienia programowania aplikacji wielorobotowych
			if (ecp_reply.reply != lib::ERROR_IN_ECP)
				set_ecp_reply(lib::TASK_TERMINATED);
			returned_value = false;
			break;
		default:
			set_ecp_reply(lib::INCORRECT_MP_COMMAND);
			break;
	}

	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0)

	{// by Y&W
		uint64_t e = errno; // kod bledu systemowego
		perror("ecp: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Reply to MP failed");
		throw common::robot::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	// ew. odebranie pulsu z MP
	// sprawdzeniem czy MP wyslalo puls przed spotkaniem z ECP
	if ((!mp_pulse_received) && (mp_command.pulse_to_ecp_sent)) {
		caller = receive_mp_message(true);
	}

	if (ecp_stop)
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == lib::INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n", __FILE__, __LINE__);
		throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return returned_value;
}

// Receive of mp message
int task_base::receive_mp_message(bool block)
{
	//std::cerr << "ecp receive_mp_message 1" << std::endl;

	while (1) {
		int caller = -100;

		int32_t type, subtype;
		//	std::cerr << "ecp receive_mp_message messip 2" << std::endl;
		if (block) {
			caller = messip::port_receive(ecp_attach, type, subtype, mp_command);
		} else {
			caller = messip::port_receive(ecp_attach, type, subtype, mp_command, 0);
		}
		//	std::cerr << "ecp receive_mp_message messip 3" << std::endl;


		if (caller < 0) {/* Error condition, exit */

			if (caller == MESSIP_MSG_CONNECTING) {
				//			std::cerr << "ecp receive_mp_message messip 4a" << std::endl;
				continue;
			} else if (caller == MESSIP_MSG_TIMEOUT) {
				//		std::cerr << "ecp receive_mp_message messip 4b" << std::endl;
				return MESSIP_MSG_TIMEOUT;
			} else if (caller == MESSIP_MSG_NOREPLY) {
				if (type == MP_TO_ECP_COMMUNICATION_REQUEST) {
					return caller;
				} else {
					continue;
				}
			} else if (caller == MESSIP_MSG_DISCONNECT) {

				return MESSIP_MSG_DISCONNECT;
			}

			uint64_t e = errno; // kod bledu systemowego
			std::cerr << "ecp 1b1" << caller << std::endl;
			perror("ecp: Receive from MP failed");
			std::cerr << "ecp 1b2" << std::endl;
			sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Receive from MP failed");
			throw common::robot::ECP_error(lib::SYSTEM_ERROR, 0);
		}
		//		std::cerr << "ecp receive_mp_message messip 5" << std::endl;
		if (caller < -1) {
			// ie. MESSIP_MSG_DISCONNECT
			fprintf(stderr, "mp: messip::port_receive() -> %d, ie. MESSIP_MSG_DISCONNECT\n", caller);
			continue;
		}

		//	std::cerr << "ecp receive_mp_message 7:" << caller << std::endl;
		return caller;
	}
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
