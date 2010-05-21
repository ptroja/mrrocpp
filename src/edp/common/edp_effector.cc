// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>

#else
#include "messip_dataport.h"
#endif /* !USE_MESSIP_SRR */
#include <pthread.h>
#include <errno.h>

#include "lib/mis_fun.h"
#include "edp/common/edp_effector.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
effector::effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	robot_name(l_robot_name), config(_config) {

	/* Lokalizacja procesu wywietlania komunikatow SR */
	msg = new lib::sr_edp(lib::EDP, config.value<std::string> (
			"resourceman_attach_point").c_str(),
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER,
					"sr_attach_point", UI_SECTION).c_str(), true);

	sh_msg = new lib::sr_edp(lib::EDP, config.value<std::string> (
			"resourceman_attach_point").c_str(),
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER,
					"sr_attach_point", UI_SECTION).c_str(), false);

	if (config.exists("test_mode"))
		test_mode = config.value<int> ("test_mode");
	else
		test_mode = 0;

}

effector::~effector() {
	delete msg;
	delete sh_msg;
}

/*--------------------------------------------------------------------------*/
bool effector::initialize_communication() {
	const std::string server_attach_point(config.return_attach_point_name(
			lib::configurator::CONFIG_SERVER, "resourceman_attach_point"));

#if !defined(USE_MESSIP_SRR)
	// obsluga mechanizmu sygnalizacji zajetosci sprzetu
	if (!(test_mode)) {

		const std::string hardware_busy_attach_point =
				config.value<std::string> ("hardware_busy_attach_point");

		std::string
				full_path_to_hardware_busy_attach_point("/dev/name/global/");
		full_path_to_hardware_busy_attach_point += hardware_busy_attach_point;

		// sprawdzenie czy nie jakis proces EDP nie zajmuje juz sprzetu
		if (access(full_path_to_hardware_busy_attach_point.c_str(), R_OK) == 0) {
			fprintf(stderr, "EDP: hardware busy\n");
			return false;
		}

		name_attach_t * tmp_attach = name_attach(NULL,
				hardware_busy_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL);

		if (tmp_attach == NULL) {
			msg->message(lib::SYSTEM_ERROR, errno,
					"EDP: hardware_busy_attach_point failed to attach");
			fprintf(
					stderr,
					"hardware_busy_attach_point name_attach() to %s failed: %s\n",
					hardware_busy_attach_point.c_str(), strerror(errno));
			// TODO: throw
			return false;
		}
	}
#endif /* !defined(USE_MESSIP_SRR */

	std::string full_path_to_server_attach_point("/dev/name/global/");
	full_path_to_server_attach_point += server_attach_point;

	// sprawdzenie czy nie jest juz zarejestrowany server EDP
	if (access(full_path_to_server_attach_point.c_str(), R_OK) == 0) {
		fprintf(stderr, "edp already exists() failed: %s\n", strerror(errno));
		return false;
	}

	/* Ustawienie priorytetu procesu */

	lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 2);

	server_attach =
#if !defined(USE_MESSIP_SRR)
			name_attach(NULL, server_attach_point.c_str(),
					NAME_FLAG_ATTACH_GLOBAL);
#else /* USE_MESSIP_SRR */
	messip::port_create(server_attach_point);
#endif /* USE_MESSIP_SRR */

	if (server_attach == NULL) {
		msg->message(lib::SYSTEM_ERROR, errno, "EDP: resmg failed to attach");
		fprintf(stderr, "name_attach() failed: %s\n", strerror(errno));
		return false;
	}

	msg->message("EDP loaded");

	return true;
}

void effector::establish_error(uint64_t err0, uint64_t err1) {
	reply.reply_type = lib::ERROR;
	reply.error_no.error0 = err0;
	reply.error_no.error1 = err1;
}

lib::INSTRUCTION_TYPE effector::receive_instruction(void) {
	// oczekuje na polecenie od ECP, wczytuje je oraz zwraca jego typ
	int rcvid;
	/* Oczekiwanie na polecenie od ECP */

	// bufory:
	// - polecen przysylanych z ECP
	// - polecen przysylanych z ECP dla watku trans_t
	lib::ecp_command_buffer new_ecp_command;

	/* Do your MsgReceive's here now with the chid */
	while (1) {
#if !defined(USE_MESSIP_SRR)
		rcvid = MsgReceive(server_attach->chid, &new_ecp_command,
				sizeof(lib::ecp_command_buffer), NULL);

		if (rcvid == -1) {/* Error condition, exit */
			perror("MsgReceive()");
			break;
		}

		if (rcvid == 0) {/* Pulse received */
			switch (new_ecp_command.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				/*
				 * A client disconnected all its connections (called
				 * name_close() for each name_open() of our name) or
				 * terminated
				 */
				ConnectDetach(new_ecp_command.hdr.scoid);
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

		/* name_open() sends a connect message, must EOK this */
		if (new_ecp_command.hdr.type == _IO_CONNECT) {
			MsgReply(rcvid, EOK, NULL, 0);
			continue;
		}

		/* Some other QNX IO message was received; reject it */
		if (new_ecp_command.hdr.type > _IO_BASE && new_ecp_command.hdr.type
				<= _IO_MAX) {
			MsgError(rcvid, ENOSYS);
			continue;
		}
#else /* USE_MESSIP_SRR */
		int32_t type, subtype;
		rcvid = messip::port_receive(server_attach, type, subtype, new_ecp_command);

		if (rcvid == -1)
		{/* Error condition, exit */
			perror("messip::port_receive()");
			break;
		}
		else if (rcvid < -1)
		{
			fprintf(stderr, "ie. MESSIP_MSG_DISCONNECT\n");
			continue;
		}
#endif /* USE_MESSIP_SRR */

		/* A message (presumable ours) received, handle */
		break;
	}

	caller = rcvid;

	instruction = new_ecp_command.instruction;

	instruction_deserialization();

	return instruction.instruction_type;
}

void effector::reply_to_instruction(void) {
	// Wyslanie potwierdzenia przyjecia polecenia do wykonania,
	// adekwatnej odpowiedzi na zapytanie lub
	// informacji o tym, ze przyslane polecenie nie moze byc przyjte
	// do wykonania w aktualnym stanie EDP
	// int reply_size;     // liczba bajtw wysyanej odpowiedzi

	reply_serialization();

	if (!((reply.reply_type == lib::ERROR) || (reply.reply_type
			== lib::SYNCHRO_OK)))
		reply.reply_type = real_reply_type;

#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, 0, &reply, sizeof(reply)) == -1) { // Odpowiedz dla procesu ECP badz UI by Y
#else /* USE_MESSIP_SRR */
		if (messip::port_reply(server_attach, caller, 0, reply) == -1) {
#endif /* USE_MESSIP_SRR */
		uint64_t e = errno;
		perror("Reply() to ECP failed");
		msg->message(lib::SYSTEM_ERROR, e, "Reply() to ECP failed");
		throw System_error();
	}
	real_reply_type = lib::ACKNOWLEDGE;
}

void effector::instruction_deserialization() {
}

void effector::reply_serialization() {
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

