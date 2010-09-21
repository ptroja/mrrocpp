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
namespace ui {
namespace common {
class Interface;
}

namespace tfg_and_conv {
class EcpRobot;
}
namespace irp6ot_tfg {

//
//
// KLASA UiRobotIrp6ot_tfg
//
//


class UiRobot : public common::UiRobot
{
private:

public:

	double irp6ot_tfg_current_pos[lib::irp6ot_tfg::NUM_OF_SERVOS];// pozycja biezaca
	double irp6ot_tfg_desired_pos[lib::irp6ot_tfg::NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_irp6ot_tfg_moves_open; // informacja czy okno ruchow
	bool is_wind_irp6ot_tfg_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte


	tfg_and_conv::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();
	int close_all_windows();
	int delete_ui_ecp_robot();

};

}
} //namespace ui
} //namespace mrrocpp

#endif

