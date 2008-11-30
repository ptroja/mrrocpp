// -------------------------------------------------------------------------
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------

#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_task.h"

// -------------------------------------------------------------------
mp_robot::mp_robot( ROBOT_ENUM l_robot_name, const char* _section_name, mp_task &mp_object_l) :
	ecp_mp_robot(l_robot_name),
	mp_object(mp_object_l),
	sr_ecp_msg(*(mp_object_l.sr_ecp_msg))
{
	char *node_name = mp_object.config.return_string_value("node_name", _section_name);
	nd = mp_object.config.return_node_number(node_name);
	delete[] node_name;

	char * network_ecp_attach_point;
	network_ecp_attach_point = mp_object.config.return_attach_point_name
	                           (configurator::CONFIG_SERVER, "ecp_attach_point", _section_name);

#if !defined(USE_MESSIP_SRR)
	char tmp_string[100];
	sprintf(tmp_string, "/dev/name/global/%s", network_ecp_attach_point);

	// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny ECP
	if (access(tmp_string, R_OK) == 0 ) {
		sr_ecp_msg.message("ECP already exists");
		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}
#endif

	ECP_pid = mp_object.config.process_spawn(_section_name);

	if ( ECP_pid < 0) {
		uint64_t e = errno; // kod bledu
		perror ("Failed to spawn ECP process on node\n");
		sr_ecp_msg.message(SYSTEM_ERROR, e, "MP: Failed to spawn ECP");
		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	new_pulse = false;
	robot_new_pulse_checked = false;
	communicate = true; // domyslnie robot jest aktywny

	mp_receive_pulse_struct_t input;

	// oczekiwanie na zgloszenie procesu ECP
	// ret = mp_object.mp_wait_for_name_open_ecp_pulse(&input, nd, ECP_pid);
	mp_object.mp_wait_for_name_open_ecp_pulse(&input);

	scoid = input.msg_info.scoid;

	// nawiazanie komunikacji z ECP
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	// 	printf("aa: %s\n",	_config->return_attach_point_name	(CONFIG_SERVER, "ecp_attach_point", _section_name));
#if !defined(USE_MESSIP_SRR)
	while ((ECP_fd = name_open(network_ecp_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
	while ((ECP_fd = messip_channel_connect(NULL, network_ecp_attach_point, MESSIP_NOTIMEOUT)) == NULL)
#endif
		if ((tmp++) < CONNECT_RETRY)
			usleep(1000 * CONNECT_DELAY);
		else {
			uint64_t e = errno; // kod bledu
			perror("Connect to ECP failed");
			sr_ecp_msg.message (SYSTEM_ERROR, e, "Connect to ECP failed");
			delete[] network_ecp_attach_point;
			throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		}

	delete[] network_ecp_attach_point;
}
// -------------------------------------------------------------------


mp_robot::~mp_robot() {
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
void mp_robot::start_ecp ( void ) {

	mp_command.command = START_TASK;
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
		sr_ecp_msg.message(SYSTEM_ERROR, e, "MP: Send to ECP failed");
		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	// by Y - ECP_ACKNOWLEDGE zamienione na TASK_TERMINATED
	// w celu uproszczenia oprogramowania zadan wielorobotowych
	if (ecp_reply_package.reply != TASK_TERMINATED ) {
		// Odebrano od ECP informacje o bledzie
		printf("Error w start_ecp w ECP\n");
		throw MP_main_error(NON_FATAL_ERROR, ECP_ERRORS);
	}
}
// ------------------------------------------------------------------------


// -------------------------------------------------------------------
void mp_robot::execute_motion ( void ) { // zlecenie wykonania ruchu

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
		sr_ecp_msg.message(SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error (SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_reply_package.reply == ERROR_IN_ECP ) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error (NON_FATAL_ERROR, ECP_ERRORS);
	}
	// W.S. ...
	// Ewentualna aktualizacja skladowych robota na podstawie ecp_reply
}
// ---------------------------------------------------------------



// -------------------------------------------------------------------
void mp_robot::terminate_ecp ( void ) { // zlecenie STOP zakonczenia ruchu
	mp_command.command = STOP;
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
		perror("Send to ECP failed ?\n");
		sr_ecp_msg.message(SYSTEM_ERROR, e, "MP: Send() to ECP failed");
		throw MP_error (SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_reply_package.reply == ERROR_IN_ECP) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error (NON_FATAL_ERROR, ECP_ERRORS);
	}
}
// ---------------------------------------------------------------


// --------------------------------------------------------------------------
void mp_robot::create_command (void) {
// wypelnia bufor wysylkowy do ECP na podstawie danych
// zawartych w skladowych generatora lub warunku

	mp_command.command = ecp_td.mp_command;

	switch (mp_command.command) {
		case NEXT_STATE:
			mp_command.mp_package.ecp_next_state = ecp_td.ecp_next_state;
			break;
		case NEXT_POSE:
			create_next_pose_command();
			break;
		default:
			break;
	}
}
// ---------------------------------------------------------------
void mp_robot::get_reply(void) {
	// pobiera z pakietu przeslanego z ECP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku

	ecp_td.ecp_reply = ecp_reply_package.reply;
	ecp_td.reply_type = ecp_reply_package.ecp_reply.reply_package.reply_type;

	// TODO: czy warto wprowadzac klase potomna?
	if (robot_name == ROBOT_SPEECHRECOGNITION) {
		strncpy(ecp_td.commandRecognized, ecp_reply_package.commandRecognized, SPEECH_RECOGNITION_TEXT_LEN);
	}

	switch (ecp_td.reply_type) {
		case ERROR:
			ecp_td.error_no.error0 = ecp_reply_package.ecp_reply.reply_package.error_no.error0;
			ecp_td.error_no.error1 = ecp_reply_package.ecp_reply.reply_package.error_no.error1;
			break;
		case ACKNOWLEDGE:
			break;
		default:  // bledna przesylka
			throw MP_error (NON_FATAL_ERROR, INVALID_EDP_REPLY);
	}
}
