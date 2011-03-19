// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SPKM_H
#define _UI_ECP_R_SPKM_H

#include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "robot/spkm/ecp_r_spkm.h"
#include "ui/src/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm {

// ---------------------------------------------------------------
class EcpRobot : public common::EcpRobotDataPort
{

public:
	lib::single_thread_port <lib::epos::epos_simple_command> * epos_motor_command_data_port;
	lib::single_thread_port <lib::epos::epos_simple_command> * epos_joint_command_data_port;
	lib::single_thread_port <lib::frame_tab> * epos_external_command_data_port;
	lib::single_thread_port <lib::epos::epos_cubic_command> * epos_cubic_command_data_port;
	lib::single_thread_port <lib::epos::epos_trapezoidal_command> * epos_trapezoidal_command_data_port;
	lib::single_thread_port <lib::epos::epos_operational_command> * epos_operational_command_data_port;
	lib::single_thread_port <bool> * epos_brake_command_data_port;

	lib::single_thread_request_port <lib::epos::epos_reply> * epos_reply_data_request_port;
	lib::single_thread_request_port <lib::epos::epos_reply> * epos_joint_reply_data_request_port;
	lib::single_thread_request_port <lib::epos::epos_reply> * epos_external_reply_data_request_port;

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::Interface& _interface); // Konstruktor

	void move_motors(const double final_position[lib::MAX_SERVOS_NR]);
	void move_joints(const double final_position[lib::MAX_SERVOS_NR]);
	void move_external(const double final_position[6]);
};

}
} //namespace ui
} //namespace mrrocpp

#endif

