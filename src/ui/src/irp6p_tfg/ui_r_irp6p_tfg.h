// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6P_TFG_H
#define __UI_R_IRP6P_TFG_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotIrp6p_tfg
//
//


// super klasa agregujaca porozrzucane struktury


class ui_tfg_and_conv_robot;

class UiRobotIrp6p_tfg: public UiRobot {
private:

public:

	bool is_wind_irp6p_tfg_moves_open; // informacja czy okno ruchow
	bool is_wind_irp6p_tfg_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte


	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotIrp6p_tfg();
	int reload_configuration();
	int manage_interface();
};

#endif

