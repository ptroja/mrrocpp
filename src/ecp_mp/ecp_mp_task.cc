// -------------------------------------------------------------------------
//                            ecp_mp_task.cc
//            Effector Control Process (lib::ECP) i MP - methods
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


lib::sr_ecp* task::sr_ecp_msg = NULL;

// mapa wszystkich czujnikow
sensor_map task::sensor_m;

// mapa wszystkich transmitterów
transmitter_map task::transmitter_m;

task::task(lib::configurator &_config)
	: config(_config)
{
	mrrocpp_network_path = config.return_mrrocpp_network_path();

	std::string ui_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "ui_attach_point", "[ui]");

    // kilka sekund  (~1) na otworzenie urzadzenia
    short tmp = 0;
#if !defined(USE_MESSIP_SRR)
    while ((UI_fd = name_open(ui_net_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0) {
#else
	while ((UI_fd = messip_channel_connect(NULL, ui_net_attach_point.c_str(), MESSIP_NOTIMEOUT)) == NULL) {
#endif
        if ((tmp++)<CONNECT_RETRY)
            usleep(1000*CONNECT_DELAY);
        else
        {
            int e = errno;
            perror("Connect to UI failed");
            sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "Connect to UI failed");
            throw ecp::common::ECP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
        }
    }
}


// --------------------------------------------------------------------------
// Odpowiedz operatora typu (Yes/No) na zadane pytanie (question)
bool task::operator_reaction (const char* question )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::YES_NO;     // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI podczas uczenia

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return (ui_to_ecp_rep.reply == lib::ANSWER_YES);
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// by Y - Wybor przez operatora jednej z opcji
lib::BYTE task::choose_option (const char* question, lib::BYTE nr_of_options_input )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::CHOOSE_OPTION; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI
	ecp_to_ui_msg.nr_of_options = nr_of_options_input;

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message),  &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return ui_to_ecp_rep.reply; // by Y
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby calkowitej (int)
int task::input_integer (const char* question )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::INTEGER_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message),  &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return ui_to_ecp_rep.integer_number;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby rzeczywistej (double)
double task::input_double (const char* question )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::DOUBLE_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message),  &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}
	return ui_to_ecp_rep.double_number; // by Y
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Informacja wymagajaca potwierdzenia odbioru przez operatora
bool task::show_message (const char* message)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::MESSAGE; // Polecenie wyswietlenia komunikatu
	strcpy(ecp_to_ui_msg.string, message);

	ecp_to_ui_msg.hdr.type=0;
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message),  &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return (ui_to_ecp_rep.reply == lib::ANSWER_YES);
}
// --------------------------------------------------------------------------


// Funkcje do obslugi czujnikow

// ------------------------------------------------------------------------
void task::kill_all_VSP (std::map <lib::SENSOR_ENUM, lib::sensor*>& _sensor_m)
{
	// Zabicie wszystkich procesow VSP
	for (ecp_mp::sensor_map::iterator sensor_m_iterator = _sensor_m.begin();
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


void task::all_sensors_initiate_reading (std::map <lib::SENSOR_ENUM, lib::sensor*>& _sensor_m)
{
	for (ecp_mp::sensor_map::iterator sensor_m_iterator = _sensor_m.begin();
	        sensor_m_iterator != _sensor_m.end(); sensor_m_iterator++) {
		if (sensor_m_iterator->second->base_period > 0) {
			if (sensor_m_iterator->second->current_period == sensor_m_iterator->second->base_period) {
				sensor_m_iterator->second->initiate_reading();
			}
			sensor_m_iterator->second->current_period--;
		}
	}
}

void task::all_sensors_get_reading (std::map <lib::SENSOR_ENUM, lib::sensor*>& _sensor_m)
{

	for (ecp_mp::sensor_map::iterator sensor_m_iterator = _sensor_m.begin();
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

