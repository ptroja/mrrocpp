// -------------------------------------------------------------------------
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/datastr.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/task/mp_task.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace robot {

robot::MP_error::MP_error(lib::error_class_t err0, uint64_t err1) :
	error_class(err0), error_no(err1) {
}

// -------------------------------------------------------------------
robot::robot(lib::robot_name_t l_robot_name, const std::string & _section_name,
		task::task &mp_object_l) :
	ecp_mp::robot(l_robot_name), mp_object(mp_object_l), communicate(true), // domyslnie robot jest aktywny
			sr_ecp_msg(*(mp_object_l.sr_ecp_msg)), opened(false), new_pulse(
					false), new_pulse_checked(false), continuous_coordination(
					false) {
	mp_command.pulse_to_ecp_sent = false;

	std::string node_name(mp_object.config.value<std::string> ("node_name",
			_section_name));

#if !defined(PROCESS_SPAWN_RSH)
	nd = mp_object.config.return_node_number(node_name.c_str());
#endif

	std::string ecp_attach_point(
			mp_object.config.return_attach_point_name(
					lib::configurator::CONFIG_SERVER, "ecp_attach_point",
					_section_name));

	ECP_pid = mp_object.config.process_spawn(_section_name);

	if (ECP_pid < 0) {
		uint64_t e = errno; // kod bledu
		perror("Failed to spawn ECP");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Failed to spawn ECP");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	// handle ECP's name_open() call
	scoid = mp_object.wait_for_name_open();
	opened = true;

	// nawiazanie komunikacji z ECP
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
#if !defined(USE_MESSIP_SRR)
	while ((ECP_fd = name_open(ecp_attach_point.c_str(),
			NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
		while ((ECP_fd = messip::port_connect(ecp_attach_point)) == NULL)
#endif
		if ((tmp++) < CONNECT_RETRY)
			usleep(1000 * CONNECT_DELAY);
		else {
			uint64_t e = errno; // kod bledu
			fprintf(stderr, "Connect to ECP failed at channel '%s'\n",
					ecp_attach_point.c_str());
			perror("Connect to ECP failed");
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "Connect to ECP failed");
			throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
		}
}
// -------------------------------------------------------------------


robot::~robot() {
	fprintf(stderr, "robot::~robot()\n");
#if !defined(USE_MESSIP_SRR)
	if (ECP_fd >= 0) {
		name_close(ECP_fd);
	}
#else /* USE_MESSIP_SRR */
	if (ECP_fd) {
		messip::port_disconnect(ECP_fd);
	}
#endif /* USE_MESSIP_SRR */

#if defined(PROCESS_SPAWN_RSH)
	if (kill(ECP_pid, SIGTERM) == -1) {
		perror("kill()");
		fprintf(stderr, "kill failed for robot %s pid %d\n", lib::toString(
				robot_name).c_str(), ECP_pid);
	} else {
		if (waitpid(ECP_pid, NULL, 0) == -1) {
			perror("waitpid()");
		}
	}
#else
	SignalKill(nd, ECP_pid, 0, SIGTERM, 0, 0);
#endif
}

// Wysyla puls do Mp przed oczekiwaniem na spotkanie
void robot::send_pulse_to_ecp(int pulse_code, int pulse_value) {
	if ((!(mp_command.pulse_to_ecp_sent)) && (!new_pulse)
			&& (!continuous_coordination) && (!(mp_command.command
			== lib::NEXT_STATE))) {

#if !defined(USE_MESSIP_SRR)
		if (MsgSendPulse(ECP_fd, sched_get_priority_min(SCHED_FIFO),
				pulse_code, pulse_value) == -1)
#else
		if (messip::port_send_pulse(ECP_fd, pulse_code, pulse_value) < 0)
#endif
		{
			perror("MsgSendPulse()");
		}
		mp_command.pulse_to_ecp_sent = true;
	}
}

// ------------------------------------------------------------------------
void robot::start_ecp(void) {

	mp_command.command = lib::START_TASK;

#if !defined(USE_MESSIP_SRR)
	mp_command.hdr.type = 0;
	if (MsgSend(ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package,
			sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
		if(messip::port_send(ECP_fd, 0, 0, mp_command, ecp_reply_package) < 0) {
#endif
		uint64_t e = errno;
		perror("Send to ECP failed");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send to ECP failed");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	}
	mp_command.pulse_to_ecp_sent = false;
	// by Y - ECP_ACKNOWLEDGE zamienione na lib::TASK_TERMINATED
	// w celu uproszczenia programowania zadan wielorobotowych
	if (ecp_reply_package.reply != lib::TASK_TERMINATED) {
		// Odebrano od ECP informacje o bledzie
		printf("Error w start_ecp w ECP\n");
		throw common::MP_main_error(lib::NON_FATAL_ERROR, ECP_ERRORS);
	}
}
// ------------------------------------------------------------------------


// -------------------------------------------------------------------
void robot::execute_motion(void) { // zlecenie wykonania ruchu

#if !defined(USE_MESSIP_SRR)
	mp_command.hdr.type = 0;
	if (MsgSend(ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package,
			sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
		if(messip::port_send(ECP_fd, 0, 0, mp_command, ecp_reply_package) < 0) {
#endif
		// Blad komunikacji miedzyprocesowej - wyjatek
		uint64_t e = errno;
		perror("Send to ECP failed");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
	mp_command.pulse_to_ecp_sent = false;

	if (ecp_reply_package.reply == lib::ERROR_IN_ECP) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);
	}
	// W.S. ...
	// Ewentualna aktualizacja skladowych robota na podstawie ecp_reply
}
// ---------------------------------------------------------------


// -------------------------------------------------------------------
void robot::terminate_ecp(void) { // zlecenie STOP zakonczenia ruchu
	mp_command.command = lib::STOP;

#if !defined(USE_MESSIP_SRR)
	mp_command.hdr.type = 0;
	if (MsgSend(ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package,
			sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
		if(messip::port_send(ECP_fd, 0, 0, mp_command, ecp_reply_package) < 0) {
#endif
		// Blad komunikacji miedzyprocesowej - wyjatek
		uint64_t e = errno;
		perror("Send to ECP failed ?");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
	mp_command.pulse_to_ecp_sent = false;
	if (ecp_reply_package.reply == lib::ERROR_IN_ECP) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);
	}
}
// ---------------------------------------------------------------


} // namespace robot
} // namespace mp
} // namespace mrrocpp

