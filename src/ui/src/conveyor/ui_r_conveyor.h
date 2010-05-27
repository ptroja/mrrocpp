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

//
//
// KLASA UiRobotConveyor
//
//


// super klasa agregujaca porozrzucane struktury


class ui_tfg_and_conv_robot;

class UiRobotConveyor: public UiRobot {
private:

public:

	bool is_wind_conv_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_conveyor_moves_open; // informacja czy okno ruchow dla robota conveyor

	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotConveyor();
	int reload_configuration();
	int manage_interface();
};

#endif

