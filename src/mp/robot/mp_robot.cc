// -------------------------------------------------------------------------
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------

#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/task/mp_task.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace robot {


robot::MP_error::MP_error(lib::error_class_t err0, uint64_t err1) :
        error_class(err0), error_no(err1)
{}

// -------------------------------------------------------------------
robot::robot( lib::robot_name_t l_robot_name, const char* _section_name, task::task &mp_object_l) :
	ecp_mp::robot(l_robot_name),
	mp_object(mp_object_l),
	communicate(true), // domyslnie robot jest aktywny
	sr_ecp_msg(*(mp_object_l.sr_ecp_msg)),
	opened(false),
	new_pulse(false),
	new_pulse_checked(false)
{
	std::string node_name(mp_object.config.return_string_value("node_name", _section_name));
	nd = mp_object.config.return_node_number(node_name.c_str());

	std::string ecp_attach_point(mp_object.config.return_attach_point_name
	                           (lib::configurator::CONFIG_SERVER, "ecp_attach_point", _section_name));

	ECP_pid = mp_object.config.process_spawn(_section_name);

	if ( ECP_pid < 0) {
		uint64_t e = errno; // kod bledu
		perror ("Failed to spawn ECP");
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
	while ((ECP_fd = name_open(ecp_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
	while ((ECP_fd = messip_channel_connect(NULL, ecp_attach_point.c_str(), MESSIP_NOTIMEOUT)) == NULL)
#endif
		if ((tmp++) < CONNECT_RETRY)
			usleep(1000 * CONNECT_DELAY);
		else {
			uint64_t e = errno; // kod bledu
			fprintf(stderr, "Connect to ECP failed at channel '%s'\n", ecp_attach_point.c_str());
			perror("Connect to ECP failed");
			sr_ecp_msg.message (lib::SYSTEM_ERROR, e, "Connect to ECP failed");
			throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
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
		messip_channel_disconnect(ECP_fd, MESSIP_NOTIMEOUT);
	}
#endif /* USE_MESSIP_SRR */

#if defined(PROCESS_SPAWN_RSH)
		kill(ECP_pid, SIGTERM);
#else
		SignalKill(nd, ECP_pid, 0, SIGTERM, 0, 0);
#endif
}

// ------------------------------------------------------------------------
void robot::start_ecp ( void ) {

	mp_command.command = lib::START_TASK;

#if !defined(USE_MESSIP_SRR)
	mp_command.hdr.type = 0;
	if ( MsgSend ( ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package, sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
	int status;
	if(messip_send(ECP_fd, 0, 0, &mp_command, sizeof(mp_command),
				&status, &ecp_reply_package, sizeof(ecp_reply_package), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("Send to ECP failed");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send to ECP failed");
		throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	// by Y - ECP_ACKNOWLEDGE zamienione na lib::TASK_TERMINATED
	// w celu uproszczenia programowania zadan wielorobotowych
	if (ecp_reply_package.reply != lib::TASK_TERMINATED ) {
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
	if ( MsgSend ( ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package, sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
	int status;
	if(messip_send(ECP_fd, 0, 0, &mp_command, sizeof(mp_command),
			&status, &ecp_reply_package, sizeof(ecp_reply_package), MESSIP_NOTIMEOUT) < 0) {
#endif
		// Blad komunikacji miedzyprocesowej - wyjatek
		uint64_t e = errno;
		perror("Send to ECP failed");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error (lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_reply_package.reply == lib::ERROR_IN_ECP ) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error (lib::NON_FATAL_ERROR, ECP_ERRORS);
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
	if (MsgSend(ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package, sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
	int status;
	if(messip_send(ECP_fd, 0, 0, &mp_command, sizeof(mp_command),
			&status, &ecp_reply_package, sizeof(ecp_reply_package), MESSIP_NOTIMEOUT) < 0) {
#endif
		// Blad komunikacji miedzyprocesowej - wyjatek
		uint64_t e = errno;
		perror("Send to ECP failed ?");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error (lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_reply_package.reply == lib::ERROR_IN_ECP) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error (lib::NON_FATAL_ERROR, ECP_ERRORS);
	}
}
// ---------------------------------------------------------------



} // namespace robot
} // namespace mp
} // namespace mrrocpp

