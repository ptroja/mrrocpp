// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6OT_TFG_H
#define __UI_R_IRP6OT_TFG_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotIrp6ot_tfg
//
//


// super klasa agregujaca porozrzucane struktury


class ui_tfg_and_conv_robot;

class UiRobotIrp6ot_tfg: public UiRobot {
private:

public:

	bool is_wind_irp6ot_tfg_moves_open; // informacja czy okno ruchow
	bool is_wind_irp6ot_tfg_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte


	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotIrp6ot_tfg();
	int reload_configuration();
	int manage_interface();
};

#endif

