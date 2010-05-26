// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_BIRD_HAND_H
#define _UI_ECP_R_BIRD_HAND_H

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "lib/mrmath/mrmath.h"
#include "ecp/common/ecp_robot.h"
#include "ecp/bird_hand/ecp_r_bird_hand.h"

// ---------------------------------------------------------------
class ui_bird_hand_robot {

public:

	// zadawanie nastaw regulatorow
	mrrocpp::lib::single_thread_port<mrrocpp::lib::bird_hand_command>
			*bird_hand_command_data_port;
	mrrocpp::lib::bird_hand_command bird_hand_command_structure;

	// zadawanie parametrow konfiguracji
	lib::single_thread_port<lib::bird_hand_configuration>
			*bird_hand_configuration_command_data_port;
	lib::bird_hand_configuration bird_hand_configuration_command_structure;

	// odbieranie statusu robota
	lib::single_thread_request_port<lib::bird_hand_status>
			*bird_hand_status_reply_data_request_port;
	lib::bird_hand_status bird_hand_status_reply_structure;

	// odczytanie parametrow konfiguracji
	lib::single_thread_request_port<lib::bird_hand_configuration>
			*bird_hand_configuration_reply_data_request_port;
	lib::bird_hand_configuration bird_hand_configuration_reply_structure;

	ecp::bird_hand::robot *the_robot;

	// by Y - do odczytu stanu poczatkowego robota
	void get_controller_state(
			lib::controller_state_t & robot_controller_initial_state_l);
	virtual void execute_motion(void);

	// ecp_buffer ui_edp_package; // by Y
	ui_bird_hand_robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg); // Konstruktor

	virtual ~ui_bird_hand_robot();

};
#endif
