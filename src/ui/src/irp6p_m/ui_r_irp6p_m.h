// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6P_M_H
#define __UI_R_IRP6P_M_H

#include "ui/ui.h"
#include "ui/ui_robot.h"
#include "lib/robot_consts/irp6p_m_const.h"

//
//
// KLASA UiRobotIrp6p_m
//
//


// super klasa agregujaca porozrzucane struktury


class ui_irp6_common_robot;

class UiRobotIrp6p_m: public UiRobot {
private:

public:

	double irp6p_current_pos[IRP6P_M_NUM_OF_SERVOS]; // pozycja biezaca
	double irp6p_desired_pos[IRP6P_M_NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_irp6p_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_irp6p_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_irp6p_xyz_euler_zyz_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_angle_axis_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_aa_relative_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_angle_axis_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_euler_zyz_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_kinematic_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6p_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte


	ui_irp6_common_robot *ui_ecp_robot;

	UiRobotIrp6p_m();
	int reload_configuration();
	int manage_interface();
	int process_control_window_irp6p_section_init(
			bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
			bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
			bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger);
};

#endif

