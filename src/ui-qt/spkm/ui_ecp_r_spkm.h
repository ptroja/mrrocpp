// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SPKM_H
#define _UI_ECP_R_SPKM_H

#include "../base/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "robot/spkm/ecp_r_spkm.h"
#include "../base/ui_ecp_robot/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm {

// ---------------------------------------------------------------
class EcpRobot : public common::_EcpRobotDataPort <ecp::spkm::robot>
{
public:
	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::UiRobot& _ui_robot); // Konstruktor

	void
	move_motors(const double final_position[lib::spkm::NUM_OF_SERVOS], lib::epos::EPOS_MOTION_VARIANT motion_variant);
	void
	move_joints(const double final_position[lib::spkm::NUM_OF_SERVOS], lib::epos::EPOS_MOTION_VARIANT motion_variant);
	void
	move_external(const double final_position[6], lib::epos::EPOS_MOTION_VARIANT motion_variant, lib::spkm::POSE_SPECIFICATION tool_variant, const double _estimated_time);
	void clear_fault();
	void stop_motors();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

