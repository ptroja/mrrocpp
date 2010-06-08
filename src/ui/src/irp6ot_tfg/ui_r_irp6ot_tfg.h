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
#include "lib/robot_consts/irp6ot_tfg_const.h"

//
//
// KLASA UiRobotIrp6ot_tfg
//
//


// super klasa agregujaca porozrzucane struktury

class Ui;
class ui_tfg_and_conv_robot;

class UiRobotIrp6ot_tfg: public UiRobot {
private:

public:

	double irp6ot_tfg_current_pos[IRP6OT_TFG_NUM_OF_SERVOS];// pozycja biezaca
	double irp6ot_tfg_desired_pos[IRP6OT_TFG_NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_irp6ot_tfg_moves_open; // informacja czy okno ruchow
	bool is_wind_irp6ot_tfg_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte


	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotIrp6ot_tfg(Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int close_all_windows();
	int delete_ui_ecp_robot();

};

#endif

