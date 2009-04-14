// -------------------------------------------------------------------------
//                            ecp_mp_task.cc
//            Effector Control Process (ECP) i MP - methods
//
// -------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif /* __QNXNTO__ */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_task.h"
#include "ecp_mp/ecp_mp_sensor.h"
#include "ecp/common/ECP_main_error.h"

namespace mrrocpp {
namespace ecp_mp {
namespace task {


lib::sr_ecp* base::sr_ecp_msg = NULL;

// mapa wszystkich czujnikow
std::map <lib::SENSOR_ENUM, lib::sensor*> base::sensor_m;
std::map <transmitter::TRANSMITTER_ENUM, transmitter::base*> base::transmitter_m;

base::base(lib::configurator &_config)
	: config(_config)
{
	mrrocpp_network_path = config.return_mrrocpp_network_path();

	char* ui_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "ui_attach_point", "[ui]");

    // kilka sekund  (~1) na otworzenie urzadzenia
    short tmp = 0;
#if !defined(USE_MESSIP_SRR)
    while ((UI_fd = name_open(ui_net_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0) {
#else
	while ((UI_fd = messip_channel_connect(NULL, ui_net_attach_point, MESSIP_NOTIMEOUT)) == NULL) {
#endif
        if ((tmp++)<CONNECT_RETRY)
            usleep(1000*CONNECT_DELAY);
        else
        {
            int e = errno;
            perror("Connect to UI failed");
            sr_ecp_msg->message (SYSTEM_ERROR, e, "Connect to UI failed");
            throw ecp::common::ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
        }
    }
	


    delete [] ui_net_attach_point;
}

base::~base()
{
	delete [] mrrocpp_network_path;
}


// --------------------------------------------------------------------------
// Odpowiedz operatora typu (Yes/No) na zadane pytanie (question)
bool base::operator_reaction (const char* question )
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = YES_NO;     // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI podczas uczenia

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(ECP_message), &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(ECP_message),
					&status, &ui_to_ecp_rep, sizeof(UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(SYSTEM_ERROR, 0);
	}

	return (ui_to_ecp_rep.reply == ANSWER_YES);
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// by Y - Wybor przez operatora jednej z opcji
BYTE base::choose_option (const char* question, BYTE nr_of_options_input )
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = CHOOSE_OPTION; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI
	ecp_to_ui_msg.nr_of_options = nr_of_options_input;

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(ECP_message),
					&status, &ui_to_ecp_rep, sizeof(UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(SYSTEM_ERROR, 0);
	}

	return ui_to_ecp_rep.reply; // by Y
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby calkowitej (int)
int base::input_integer (const char* question )
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = INTEGER_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(ECP_message),
					&status, &ui_to_ecp_rep, sizeof(UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(SYSTEM_ERROR, 0);
	}

	return ui_to_ecp_rep.integer_number;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby rzeczywistej (double)
double base::input_double (const char* question )
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = DOUBLE_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(ECP_message),
					&status, &ui_to_ecp_rep, sizeof(UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(SYSTEM_ERROR, 0);
	}
	return ui_to_ecp_rep.double_number; // by Y
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Informacja wymagajaca potwierdzenia odbioru przez operatora
bool base::show_message (const char* message)
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = MESSAGE; // Polecenie wyswietlenia komunikatu
	strcpy(ecp_to_ui_msg.string, message);

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(ECP_message),
					&status, &ui_to_ecp_rep, sizeof(UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(SYSTEM_ERROR, 0);
	}

	return (ui_to_ecp_rep.reply == ANSWER_YES);
}
// --------------------------------------------------------------------------


// Funkcje do obslugi czujnikow

// ------------------------------------------------------------------------
void base::kill_all_VSP (std::map <lib::SENSOR_ENUM, lib::sensor*>& _sensor_m)
{
	// Zabicie wszystkich procesow VSP
	for (std::map <lib::SENSOR_ENUM, lib::sensor*>::iterator sensor_m_iterator = _sensor_m.begin();
	        sensor_m_iterator != _sensor_m.end(); sensor_m_iterator++) {
		if (sensor_m_iterator->second->pid !=0) {
#if defined(PROCESS_SPAWN_RSH)
			kill(sensor_m_iterator->second->pid, SIGTERM);
#else
			SignalKill(lib::configurator::return_node_number(sensor_m_iterator->second->node_name),
			           sensor_m_iterator->second->pid, 0, SIGTERM, 0, 0);
#endif
		}
	}
}
// ------------------------------------------------------------------------


void base::all_sensors_initiate_reading (std::map <lib::SENSOR_ENUM, lib::sensor*>& _sensor_m)
{
	for (std::map <lib::SENSOR_ENUM, lib::sensor*>::iterator sensor_m_iterator = _sensor_m.begin();
	        sensor_m_iterator != _sensor_m.end(); sensor_m_iterator++) {
		if (sensor_m_iterator->second->base_period > 0) {
			if (sensor_m_iterator->second->current_period == sensor_m_iterator->second->base_period) {
				sensor_m_iterator->second->initiate_reading();
			}
			sensor_m_iterator->second->current_period--;
		}
	}
}

void base::all_sensors_get_reading (std::map <lib::SENSOR_ENUM, lib::sensor*>& _sensor_m)
{

	for (std::map <lib::SENSOR_ENUM, lib::sensor*>::iterator sensor_m_iterator = _sensor_m.begin();
	        sensor_m_iterator != _sensor_m.end(); sensor_m_iterator++) {
		// jesli wogole mamy robic pomiar
		if (sensor_m_iterator->second->base_period > 0) {
			if (sensor_m_iterator->second->current_period == 0) {
				sensor_m_iterator->second->get_reading();
				sensor_m_iterator->second->current_period = sensor_m_iterator->second->base_period;
			}
		}
	}
}

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

