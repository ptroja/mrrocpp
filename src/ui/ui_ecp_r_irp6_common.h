
// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_MANIP_H
#define _UI_ECP_R_MANIP_H

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "lib/mrmath/mrmath.h"
#include "ui/ui_ecp_r_common.h"

#include "ecp/common/ecp_robot.h"

// ---------------------------------------------------------------
class ui_irp6_common_robot : public ui_common_robot
{
public:
	// ecp_buffer ui_edp_package; // by Y
	ui_irp6_common_robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name); // Konstruktor


	// Zlecenie ruchu
	void move_motors(const double final_position[MAX_SERVOS_NR]);
	void move_joints(const double final_position[MAX_SERVOS_NR]);
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
#endif
