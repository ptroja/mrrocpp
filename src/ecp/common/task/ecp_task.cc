#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <stdio.h>

#include "lib/mis_fun.h"
#include "ecp/common/task/ecp_task.h"
#include "ecp/common/ECP_main_error.h"
#include "ecp/common/generator/ecp_generator.h"

#if defined(USE_MESSIP_SRR)
#include <messip_dataport.h>
#endif

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

task::task(lib::configurator &_config) :
	ecp_mp::task::task(_config),
	ecp_m_robot(NULL)
{
	initialize_communication();
}

task::~task()
{
	// TODO: error check
#if !defined(USE_MESSIP_SRR)
	name_detach(trigger_attach, 0);
	name_detach(ecp_attach, 0);
	name_close(MP_fd);
#else
	messip::port_delete(trigger_attach);
	messip::port_delete(ecp_attach);
	messip::port_delete(MP_fd);
#endif
}

bool task::pulse_check()
{
#if !defined(USE_MESSIP_SRR)
	struct sigevent stop_event;

	_pulse_msg ui_msg; // wiadomosc z ui

	stop_event.sigev_notify = SIGEV_UNBLOCK;// by Y zamiast creceive
	TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, &stop_event, NULL, NULL); // czekamy na odbior pulsu stopu
	int rcvid = MsgReceive(trigger_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

	if (rcvid == -1) {/* Error condition, exit */
		// perror("blad receive w reader");
	}

	if (rcvid == 0) {/* Pulse received */
		switch (ui_msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				/*
				 * A client disconnected all its connections (called
				 * name_close() for each name_open() of our name) or
				 * terminated
				 */
				ConnectDetach(ui_msg.hdr.scoid);
				break;
			case _PULSE_CODE_UNBLOCK:
				/*
				 * REPLY blocked client wants to unblock (was hit by a signal or timed out).  It's up to you if you
				 * reply now or later.
				 */
				break;
			default:
				if (ui_msg.hdr.code == ECP_TRIGGER) { // odebrano puls ECP_TRIGGER
					return true;
				}
				/*
				 * A pulse sent by one of your processes or a _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
				 * from the kernel?
				 */
		}
	}

	if (rcvid > 0) {
		/* A QNX IO message received, reject */
		if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
		} else {
			/* A message (presumable ours) received, handle */
			printf("ECP trigger server receive strange message of type: %d\n", ui_msg.data);
			MsgReply(rcvid, EOK, 0, 0);
		}
	}

	return false;
#else
	int32_t type, subtype;
	int recvid;
	if ((recvid = messip::port_receive_pulse(trigger_attach, type, subtype)) == -1) {
		perror("messip::port_receive()");
		return false;
	}

	if (recvid == MESSIP_MSG_NOREPLY && type == ECP_TRIGGER) {
		return true;
	}

	return false;
#endif
}


// ---------------------------------------------------------------
void task::initialize_communication()
{
	std::string mp_pulse_attach_point =
				config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_pulse_attach_point", MP_SECTION);

	std::string ecp_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "ecp_attach_point");
	std::string sr_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);

	// Obiekt do komuniacji z SR
	sr_ecp_msg = new lib::sr_ecp(lib::ECP, ecp_attach_point.c_str(), sr_net_attach_point.c_str(), true);

	//	std::cout << "ECP: Opening MP pulses channel at '" << mp_pulse_attach_point << "'" << std::endl;

#if !defined(USE_MESSIP_SRR)
	if ( (MP_fd = name_open(mp_pulse_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
	if ( (MP_fd = messip::port_connect(mp_pulse_attach_point)) == NULL)
#endif
	{
		int e = errno; // kod bledu systemowego
		fprintf(stderr, "ECP: Unable to locate MP_MASTER process at '%s'\n", mp_pulse_attach_point.c_str());
		perror("ECP: Unable to locate MP_MASTER process");
		throw ECP_main_error(lib::SYSTEM_ERROR, e);
	}

	// Rejstracja procesu ECP
#if !defined(USE_MESSIP_SRR)
	if ((ecp_attach = name_attach(NULL, ecp_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
#else
	if ((ecp_attach = messip::port_create(ecp_attach_point)) == NULL)
#endif
	{
		int e = errno; // kod bledu systemowego
		perror("Failed to attach Effector Control Process");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	std::string trigger_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point");

#if !defined(USE_MESSIP_SRR)
	if ((trigger_attach = name_attach(NULL, trigger_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
#else
	if ((trigger_attach = messip::port_create(trigger_attach_point)) == NULL)
#endif
	{
		int e = errno; // kod bledu systemowego
		perror("Failed to attach TRIGGER pulse chanel for ecp");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "Failed  Failed to name attach (trigger pulse)");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}
}
// -------------------------------------------------------------------

// Badanie typu polecenia z MP
lib::MP_COMMAND task::mp_command_type(void) const
{
	return mp_command.command;
}

// Ustawienie typu odpowiedzi z ECP do MP
void task::set_ecp_reply(lib::ECP_REPLY ecp_r)
{
	ecp_reply.reply = ecp_r;
}

// Informacja dla MP o zakonczeniu zadania uzytkownika
void task::ecp_termination_notice(void)
{
	if (mp_command_type() != lib::END_MOTION) {

		set_ecp_reply(lib::TASK_TERMINATED);
		mp_buffer_receive_and_send();
	}
}

// Wysyla puls do Mp przed oczekiwaniem na spotkanie
void task::send_pulse_to_mp(int pulse_code, int pulse_value)
{
#if !defined(USE_MESSIP_SRR)
	if (MsgSendPulse(MP_fd, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value) == -1)
#else
	if (messip::port_send_pulse(MP_fd, pulse_code, pulse_value) < 0)
#endif
	{
		perror("MsgSendPulse()");
	}
}

// Petla odbierania wiadomosci.
void task::ecp_wait_for_stop(void)
{
	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_STOP);

	// Oczekiwanie na wiadomosc.
	int caller = receive_mp_message();

	if (mp_command_type() == lib::STOP) {
		set_ecp_reply(lib::ECP_ACKNOWLEDGE);
	} else {
		set_ecp_reply(lib::ERROR_IN_ECP);
	}

	// Wyslanie odpowiedzi.
#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, EOK, &ecp_reply, sizeof(ecp_reply)) ==-1)
#else
	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0)
#endif
	{// by Y&W
		uint64_t e= errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw common::generator::generator::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	if (mp_command_type() != lib::STOP) {
		fprintf(stderr, "ecp_generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
		__FILE__, __LINE__);
		throw common::generator::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
}

// Oczekiwanie na polecenie START od MP
bool task::ecp_wait_for_start(void)
{
	bool ecp_stop = false;

	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_START);

	int caller = receive_mp_message();

	switch (mp_command_type() ) {
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

	// if (Reply (caller, &ecp_reply, sizeof(lib::ECP_REPLY_PACKAGE)) == -1 ) {
#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, EOK, &ecp_reply, sizeof(lib::ECP_REPLY_PACKAGE)) ==-1)
#else
	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0)
#endif
	{// by Y&W
		uint64_t e= errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}
	if (ecp_stop)
		throw common::generator::generator::ECP_error (lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == lib::INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
		__FILE__, __LINE__);
		throw common::generator::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	sr_ecp_msg->message("ECP user program is running");
	return false;
}

// Oczekiwanie na kolejne zlecenie od MP
void task::get_next_state(void)
{
	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_NEXT_STATE);

	int caller = receive_mp_message();

	bool ecp_stop = false;

	switch (mp_command_type() ) {
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

#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, EOK, &ecp_reply, sizeof(lib::ECP_REPLY_PACKAGE)) ==-1)
#else
	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0)
#endif
	{// by Y&W{
		uint64_t e = errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	if (ecp_stop)
		throw common::generator::generator::ECP_error (lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == lib::INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d, mp_command_type() = %d\n",
				__FILE__, __LINE__, mp_command_type());
		throw common::generator::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
}

// Oczekiwanie na polecenie od MP
bool task::mp_buffer_receive_and_send(void)
{
	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_COMMAND);

	int caller = receive_mp_message();

	bool returned_value = true;
	bool ecp_stop = false;

	switch (mp_command_type()) {
		case lib::NEXT_POSE:
			if ((ecp_reply.reply != lib::TASK_TERMINATED)&&(ecp_reply.reply != lib::ERROR_IN_ECP))
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
#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, EOK, &ecp_reply, sizeof(ecp_reply)) ==-1)
#else
	if (messip::port_reply(ecp_attach, caller, 0, ecp_reply) < 0)
#endif
	{// by Y&W
		uint64_t e= errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ecp_robot::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	if (ecp_stop)
		throw common::generator::generator::ECP_error (lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == lib::INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
		__FILE__, __LINE__);
		throw common::generator::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return returned_value;
}

// Receive of mp message
int task::receive_mp_message(void)
{
	while (1) {
#if !defined(USE_MESSIP_SRR)
		int caller = MsgReceive(ecp_attach->chid, &mp_command, sizeof(mp_command), NULL);
#else
		int32_t type, subtype;
		int caller = messip::port_receive(ecp_attach, type, subtype, mp_command);
#endif

		if (caller == -1) {/* Error condition, exit */
			uint64_t e= errno; // kod bledu systemowego
			perror("ECP: Receive from MP failed");
			sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Receive from MP failed");
			throw ecp_robot::ECP_error(lib::SYSTEM_ERROR, 0);
		}
#if !defined(USE_MESSIP_SRR)
		if (caller == 0) {/* Pulse received */
			switch (mp_command.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					/*
					 * A client disconnected all its connections (called
					 * name_close() for each name_open() of our name) or
					 * terminated
					 */
					ConnectDetach(mp_command.hdr.scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					/*
					 * REPLY blocked client wants to unblock (was hit by
					 * a signal or timed out).  It's up to you if you
					 * reply now or later.
					 */
					break;
				default:
					/*
					 * A pulse sent by one of your processes or a
					 * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
					 * from the kernel?
					 */
					break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (mp_command.hdr.type >= _IO_BASE && mp_command.hdr.type <= _IO_MAX) {
			MsgReply(caller, EOK, 0, 0);
			continue;
		}
#else
		if (caller < -1) {
			// ie. MESSIP_MSG_DISCONNECT
			fprintf(stderr, "MP: messip::port_receive() -> %d, ie. MESSIP_MSG_DISCONNECT\n", caller);
			continue;
		}
#endif
		return caller;
	}
}

ecp_sub_task::ecp_sub_task(task &_ecp_t) :
	ecp_t(_ecp_t)
{
}



} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
