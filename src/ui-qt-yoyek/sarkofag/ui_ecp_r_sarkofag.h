// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 17.02.2011
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SARKOFAG_H
#define _UI_ECP_R_SARKOFAG_H

#include "../base/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "robot/sarkofag/ecp_r_sarkofag.h"
#include "../base/ui_ecp_robot/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace sarkofag {

// ---------------------------------------------------------------
class EcpRobot : public common::_EcpRobotDataPort<ecp::sarkofag::robot>
{
public:
	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::Interface& _interface); // Konstruktor

//	void move_motors(const double final_position[lib::sarkofag::NUM_OF_SERVOS], lib::epos::EPOS_MOTION_VARIANT motion_variant);
//	void move_joints(const double final_position[lib::sarkofag::NUM_OF_SERVOS], lib::epos::EPOS_MOTION_VARIANT motion_variant);
//	void move_external(const double final_position[6], lib::epos::EPOS_MOTION_VARIANT motion_variant);
//	void clear_fault();
//	void stop_motors();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

//505454266

