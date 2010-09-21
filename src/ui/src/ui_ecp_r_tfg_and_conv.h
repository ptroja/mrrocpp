// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_TFG_H
#define _UI_ECP_R_TFG_H

#include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "ui/src/ui_ecp_r_common.h"

namespace mrrocpp {
namespace ui {
namespace tfg_and_conv {

// ---------------------------------------------------------------
class EcpRobot : public common::EcpRobot
{

public:

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name); // Konstruktor

	void move_motors(const double final_position[lib::MAX_SERVOS_NR]);
	void move_joints(const double final_position[lib::MAX_SERVOS_NR]);

};

}
} //namespace ui
} //namespace mrrocpp
#endif
