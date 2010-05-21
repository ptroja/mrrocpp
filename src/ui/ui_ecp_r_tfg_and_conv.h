// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_TFG_H
#define _UI_ECP_R_TFG_H

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "lib/mrmath/mrmath.h"
#include "ecp/common/ecp_robot.h"
#include "ui/ui_ecp_r_common.h"

// ---------------------------------------------------------------
class ui_tfg_and_conv_robot : public ui_common_robot
{

public:

	// ecp_buffer ui_edp_package; // by Y
	ui_tfg_and_conv_robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name); // Konstruktor

	void move_motors(const double final_position[MAX_SERVOS_NR]);
	void move_joints(const double final_position[MAX_SERVOS_NR]);

};
#endif
