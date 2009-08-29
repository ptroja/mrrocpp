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
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>

#include "lib/mis_fun.h"
#include "edp/common/edp_effector.h"

#include "messip/messip.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
effector::effector(lib::configurator &_config, lib::ROBOT_ENUM l_robot_name) :
	config(_config), robot_name(l_robot_name)
{

	/* Lokalizacja procesu wywietlania komunikatow SR */
	msg = new lib::sr_edp(lib::EDP, config.return_string_value("resourceman_attach_point").c_str(),
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", "[ui]").c_str());

	if (config.exists("test_mode"))
		test_mode = config.return_int_value("test_mode");
	else
		test_mode = 0;

	mrrocpp_network_path = config.return_mrrocpp_network_path();
}

bool effector::check_config(const std::string & s)
{
	return (config.exists(s.c_str()) && config.return_int_value(s.c_str()));
}

/*--------------------------------------------------------------------------*/
bool effector::initialize_communication()
{
	std::string server_attach_point(
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point"));

#if !defined(USE_MESSIP_SRR)
	// obsluga mechanizmu sygnalizacji zajetosci sprzetu
	if (!(test_mode)) {
		const char* hardware_busy_attach_point;
		char full_path_to_hardware_busy_attach_point[100];
		name_attach_t *tmp_attach;

		std::string hbap = config.return_string_value("hardware_busy_attach_point");
		hardware_busy_attach_point = hbap.c_str();
		sprintf(full_path_to_hardware_busy_attach_point, "/dev/name/global/%s", hardware_busy_attach_point);

		// sprawdzenie czy nie jakis proces EDP nie zajmuje juz sprzetu
		if (access(full_path_to_hardware_busy_attach_point, R_OK)== 0) {
			fprintf( stderr, "EDP: hardware busy\n");

			return false;
		}

		tmp_attach
				= name_attach(NULL, hardware_busy_attach_point, NAME_FLAG_ATTACH_GLOBAL);

		if (tmp_attach == NULL) {
			msg->message(lib::SYSTEM_ERROR, errno, "EDP: hardware_busy_attach_point failed to attach");
			fprintf( stderr, "hardware_busy_attach_point name_attach() to %s failed: %s\n", hardware_busy_attach_point, strerror( errno ));

			return false;
		}
	}
#endif /* !defined(USE_MESSIP_SRR */

	std::string full_path_to_server_attach_point("/dev/name/global/");
	full_path_to_server_attach_point += server_attach_point;

	// sprawdzenie czy nie jest juz zarejestrowany server EDP
	if (access(full_path_to_server_attach_point.c_str(), R_OK)== 0) {
		fprintf( stderr, "edp already exists() failed: %s\n", strerror( errno ));
		return false;
	}

	/* Ustawienie priorytetu procesu */

	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-2);

	attach =
#if !defined(USE_MESSIP_SRR)
		name_attach(NULL, server_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL);
#else /* USE_MESSIP_SRR */
		messip_channel_create(NULL, server_attach_point.c_str(), MESSIP_NOTIMEOUT, 0);
#endif /* USE_MESSIP_SRR */

	if (attach == NULL) {
		msg->message(lib::SYSTEM_ERROR, errno, "EDP: resmg failed to attach");
		fprintf( stderr, "name_attach() failed: %s\n", strerror( errno ));
		return false;
	}

	msg->message("EDP loaded");

	return true;
}


void effector::insert_reply_type(lib::REPLY_TYPE rt)
{
	reply.reply_type = rt;
}

bool effector::is_reply_type_ERROR() const
{
	return (reply.reply_type==lib::ERROR);
}

void effector::main_loop()
{
}

void effector::establish_error(uint64_t err0, uint64_t err1)
{
	reply.reply_type = lib::ERROR;
	reply.error_no.error0 = err0;
	reply.error_no.error1 = err1;
}

// lib::r_buffer
lib::REPLY_TYPE effector::is_reply_type(void) const
{
	return reply.reply_type;
}

uint64_t effector::is_error_no_0(void) const
{
	return reply.error_no.error0;
}

uint64_t effector::is_error_no_1(void) const
{
	return reply.error_no.error1;
}


lib::INSTRUCTION_TYPE effector::receive_instruction(void)
{
	// oczekuje na polecenie od ECP, wczytuje je oraz zwraca jego typ
	int rcvid;
	/* Oczekiwanie na polecenie od ECP */

	/* Do your MsgReceive's here now with the chid */
	while (1) {
#if !defined(USE_MESSIP_SRR)
		rcvid
				= MsgReceive(attach->chid, &new_ecp_command, sizeof(lib::ecp_command_buffer), NULL);

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
		rcvid = messip_receive(attach, &type, &subtype, &new_ecp_command, sizeof(lib::ecp_command_buffer), MESSIP_NOTIMEOUT);

		if (rcvid == -1)
		{/* Error condition, exit */
			perror("messip_receive()");
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

	//memcpy( &new_instruction, msg_cb, sizeof(*msg_cb) );
	caller = rcvid;
//	printf("edp instruction_type: %d\n", new_ecp_command.instruction.instruction_type);
// flushall();
	memcpy( &(new_instruction), &(new_ecp_command.instruction), sizeof(lib::c_buffer) );

	return new_instruction.instruction_type;
}

void effector::reply_to_instruction(void)
{
	// Wyslanie potwierdzenia przyjecia polecenia do wykonania,
	// adekwatnej odpowiedzi na zapytanie lub
	// informacji o tym, ze przyslane polecenie nie moze byc przyjte
	// do wykonania w aktualnym stanie EDP
	// int reply_size;     // liczba bajtw wysyanej odpowiedzi
	if ( !( (reply.reply_type == lib::ERROR) || (reply.reply_type == lib::SYNCHRO_OK) ))
		reply.reply_type = real_reply_type;
#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, 0, &reply, sizeof(reply)) == -1) { // Odpowiedz dla procesu ECP badz UI by Y
#else /* USE_MESSIP_SRR */
	if (messip_reply(attach, caller, 0, &reply, sizeof(reply), MESSIP_NOTIMEOUT) == -1) {
#endif /* USE_MESSIP_SRR */
		uint64_t e= errno;
		perror("Reply() to ECP failed");
		msg->message(lib::SYSTEM_ERROR, e, "Reply() to ECP failed");
		throw System_error();
	}
	real_reply_type = lib::ACKNOWLEDGE;
}


} // namespace common
} // namespace edp
} // namespace mrrocpp

