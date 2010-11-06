// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_MANIP_H
#define _UI_ECP_R_MANIP_H

#include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "ui/src/ui_ecp_r_common.h"

#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ui {
namespace irp6 {

// ---------------------------------------------------------------
class EcpRobot : public common::EcpRobot
{
public:
	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name); // Konstruktor


	// Zlecenie ruchu
	void move_motors(const double final_position[lib::MAX_SERVOS_NR]);
	void move_joints(const double final_position[lib::MAX_SERVOS_NR]);
	void move_xyz_euler_zyz(const double final_position[7]);
	void move_xyz_angle_axis(const double final_position[7]);
	void move_xyz_angle_axis_relative(const double position_increment[7]);
	void set_tool_xyz_angle_axis(const lib::Xyz_Angle_Axis_vector &tool_vector);
	void set_tool_xyz_euler_zyz(const lib::Xyz_Euler_Zyz_vector &tool_vector);

	// Odczyt polozenia
	void read_xyz_euler_zyz(double current_position[7]);
	void read_xyz_angle_axis(double current_position[7]);
	void read_tool_xyz_angle_axis(lib::Xyz_Angle_Axis_vector & tool_vector);
	void read_tool_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector &tool_vector);

};

}
} //namespace ui
} //namespace mrrocpp

#endif
