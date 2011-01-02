// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_POLYCRANK_H
#define __UI_R_POLYCRANK_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "robot/polycrank/const_polycrank.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace irp6 {
class EcpRobot;
}
namespace polycrank {

//
//
// KLASA UiRobotPolycrank
//
//


class UiRobot : public common::UiRobot
{
private:

public:
	double current_pos[lib::polycrank::NUM_OF_SERVOS]; // pozycja biezaca
	double desired_pos[lib::polycrank::NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_polycrank_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	//bool is_wind_polycrank_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte

	irp6::EcpRobot *ui_ecp_robot;
	//polycrank::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	void close_all_windows();
	int reload_configuration();
	int manage_interface();
	void delete_ui_ecp_robot();
	int synchronise();
	//int synchronise_int();

	void edp_create();
	int edp_create_int();

/*
	tfg_and_conv::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	void close_all_windows();
	int reload_configuration();
	int manage_interface();
	void delete_ui_ecp_robot();
	int synchronise();
	int edp_create();
	int edp_create_int();
*/

};

/*
public:
	double current_pos[lib::irp6ot_m::NUM_OF_SERVOS]; // pozycja biezaca
	double desired_pos[lib::irp6ot_m::NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_irp6ot_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_irp6ot_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_irp6ot_xyz_euler_zyz_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6ot_xyz_angle_axis_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6ot_xyz_aa_relative_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6ot_xyz_angle_axis_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6ot_xyz_euler_zyz_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6ot_kinematic_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6ot_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte

	irp6::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();
	int
			process_control_window_irp6ot_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger);
	void close_all_windows();
	void delete_ui_ecp_robot();
	int synchronise();
	int synchronise_int();
	int edp_create();
	int edp_create_int();

	int execute_motor_motion();
	int execute_joint_motion();
*/

}
} //namespace ui
} //namespace mrrocpp

#endif

