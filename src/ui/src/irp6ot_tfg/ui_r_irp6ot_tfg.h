// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6OT_TFG_H
#define __UI_R_IRP6OT_TFG_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"

namespace mrrocpp {
namespace uin {
namespace common {
class Ui;
class ui_tfg_and_conv_robot;
}
namespace irp6ot_tfg {

//
//
// KLASA UiRobotIrp6ot_tfg
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot : public common::UiRobot
{
private:

public:

	double irp6ot_tfg_current_pos[lib::irp6ot_tfg::NUM_OF_SERVOS];// pozycja biezaca
	double irp6ot_tfg_desired_pos[lib::irp6ot_tfg::NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_irp6ot_tfg_moves_open; // informacja czy okno ruchow
	bool is_wind_irp6ot_tfg_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte


	common::ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobot(common::Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int close_all_windows();
	int delete_ui_ecp_robot();

};

}
} //namespace uin
} //namespace mrrocpp

#endif

