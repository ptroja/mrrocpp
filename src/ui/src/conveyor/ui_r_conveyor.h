// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_CONVEYOR_H
#define __UI_R_CONVEYOR_H

#include "ui/ui.h"
#include "ui/ui_robot.h"
#include "lib/robot_consts/conveyor_const.h"

class Ui;

//
// KLASA UiRobotConveyor
//
//


// super klasa agregujaca porozrzucane struktury


class ui_tfg_and_conv_robot;

class UiRobotConveyor: public UiRobot {
private:

public:

	double conveyor_current_pos[CONVEYOR_NUM_OF_SERVOS];// pozycja biezaca
	double conveyor_desired_pos[CONVEYOR_NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_conv_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_conveyor_moves_open; // informacja czy okno ruchow dla robota conveyor

	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotConveyor(Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int process_control_window_conveyor_section_init(
			bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
			bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
			bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger);
	int close_all_windows();
	int delete_ui_ecp_robot();

};

#endif

