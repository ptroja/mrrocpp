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
#include "mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace robot {


robot::MP_error::MP_error(lib::ERROR_CLASS err0, uint64_t err1) :
        error_class(err0), mp_error(err1)
{}

// -------------------------------------------------------------------
robot::robot( lib::ROBOT_ENUM l_robot_name, const char* _section_name, task::task &mp_object_l) :
	ecp_mp::robot(l_robot_name),
	mp_object(mp_object_l),
	sr_ecp_msg(*(mp_object_l.sr_ecp_msg))
{
	std::string node_name(mp_object.config.return_string_value("node_name", _section_name));
	nd = mp_object.config.return_node_number(node_name.c_str());

	std::string network_ecp_attach_point(mp_object.config.return_attach_point_name
	                           (lib::configurator::CONFIG_SERVER, "ecp_attach_point", _section_name));

#if !defined(USE_MESSIP_SRR)
	char tmp_string[100];
	sprintf(tmp_string, "/dev/name/global/%s", network_ecp_attach_point.c_str());

	// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny ECP
	if (access(tmp_string, R_OK) == 0 ) {
		sr_ecp_msg.message("ECP already exists");
		throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
#endif

	ECP_pid = mp_object.config.process_spawn(_section_name);

	if ( ECP_pid < 0) {
		uint64_t e = errno; // kod bledu
		perror ("Failed to spawn ECP process on node\n");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Failed to spawn ECP");
		throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	new_pulse = false;
	robot_new_pulse_checked = false;
	communicate = true; // domyslnie robot jest aktywny

	// oczekiwanie na zgloszenie procesu ECP
	scoid = mp_object.mp_wait_for_name_open();

	// nawiazanie komunikacji z ECP
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	// 	printf("aa: %s\n",	_config->return_attach_point_name	(CONFIG_SERVER, "ecp_attach_point", _section_name));
#if !defined(USE_MESSIP_SRR)
	while ((ECP_fd = name_open(network_ecp_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
	while ((ECP_fd = messip_channel_connect(NULL, network_ecp_attach_point.c_str(), MESSIP_NOTIMEOUT)) == NULL)
#endif
		if ((tmp++) < CONNECT_RETRY)
			usleep(1000 * CONNECT_DELAY);
		else {
			uint64_t e = errno; // kod bledu
			perror("Connect to ECP failed");
			sr_ecp_msg.message (lib::SYSTEM_ERROR, e, "Connect to ECP failed");
			throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
		}
}
// -------------------------------------------------------------------


robot::~robot() {
#if !defined(USE_MESSIP_SRR)
	if (ECP_fd > 0) {
		name_close(ECP_fd);
	}
#else /* USE_MESSIP_SRR */
	if (ECP_fd) {
		messip_channel_disconnect(ECP_fd, MESSIP_NOTIMEOUT);
	}
#endif /* USE_MESSIP_SRR */
}

// ------------------------------------------------------------------------
void robot::start_ecp ( void ) {

	mp_command.command = lib::START_TASK;
	mp_command.hdr.type = 0;

#if !defined(USE_MESSIP_SRR)
	if ( MsgSend ( ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package, sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
	int status;
	if(messip_send(ECP_fd, 0, 0, &mp_command, sizeof(mp_command),
				&status, &ecp_reply_package, sizeof(ecp_reply_package), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("Send to ECP failed\n");
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

	mp_command.hdr.type = 0;

#if !defined(USE_MESSIP_SRR)
	if ( MsgSend ( ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package, sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
	int status;
	if(messip_send(ECP_fd, 0, 0, &mp_command, sizeof(mp_command),
			&status, &ecp_reply_package, sizeof(ecp_reply_package), MESSIP_NOTIMEOUT) < 0) {
#endif
		// Blad komunikacji miedzyprocesowej - wyjatek
		uint64_t e = errno;
		perror("Send to ECP failed\n");
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
	mp_command.hdr.type = 0;

#if !defined(USE_MESSIP_SRR)
	if (MsgSend(ECP_fd, &mp_command, sizeof(mp_command), &ecp_reply_package, sizeof(ecp_reply_package)) == -1) {// by Y&W
#else
	int status;
	if(messip_send(ECP_fd, 0, 0, &mp_command, sizeof(mp_command),
			&status, &ecp_reply_package, sizeof(ecp_reply_package), MESSIP_NOTIMEOUT) < 0) {
#endif
		// Blad komunikacji miedzyprocesowej - wyjatek
		uint64_t e = errno;
		perror("Send to ECP failed ?\n");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error (lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_reply_package.reply == lib::ERROR_IN_ECP) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error (lib::NON_FATAL_ERROR, ECP_ERRORS);
	}
}
// ---------------------------------------------------------------


// --------------------------------------------------------------------------
void robot::create_command (void) {
// wypelnia bufor wysylkowy do ECP na podstawie danych
// zawartych w skladowych generatora lub warunku

	mp_command.command = ecp_td.mp_command;

	switch (mp_command.command) {
		case lib::NEXT_STATE:
			mp_command.ecp_next_state = ecp_td.ecp_next_state;
			break;
		case lib::NEXT_POSE:
			create_next_pose_command();
			break;
		default:
			break;
	}
}
// ---------------------------------------------------------------
void robot::get_reply(void) {
	// pobiera z pakietu przeslanego z ECP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku

	ecp_td.ecp_reply = ecp_reply_package.reply;
	ecp_td.reply_type = ecp_reply_package.reply_package.reply_type;

	// TODO: czy warto wprowadzac klase potomna?
	if (robot_name == lib::ROBOT_SPEECHRECOGNITION) {
		strncpy(ecp_td.commandRecognized, ecp_reply_package.commandRecognized, SPEECH_RECOGNITION_TEXT_LEN);
	}

	switch (ecp_td.reply_type) {
		case lib::ERROR:
			ecp_td.error_no.error0 = ecp_reply_package.reply_package.error_no.error0;
			ecp_td.error_no.error1 = ecp_reply_package.reply_package.error_no.error1;
			break;
		case lib::ACKNOWLEDGE:
			break;
		default:  // bledna przesylka
			throw MP_error (lib::NON_FATAL_ERROR, INVALID_EDP_REPLY);
	}
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

