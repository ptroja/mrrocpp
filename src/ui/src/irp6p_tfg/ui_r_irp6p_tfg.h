// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6P_TFG_H
#define __UI_R_IRP6P_TFG_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace tfg_and_conv {
class EcpRobot;
}
namespace irp6p_tfg {

//
//
// KLASA UiRobot
//
//


class UiRobot : public common::UiRobot
{
private:

public:

	double current_pos[lib::irp6p_tfg::NUM_OF_SERVOS];// pozycja biezaca
	double desired_pos[lib::irp6p_tfg::NUM_OF_SERVOS]; // pozycja zadana


	bool is_wind_irp6p_tfg_moves_open; // informacja czy okno ruchow
	bool is_wind_irp6p_tfg_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte

	tfg_and_conv::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);

	int manage_interface();
	void close_all_windows();
	void delete_ui_ecp_robot();
	int synchronise();
	int synchronise_int();
	void edp_create();
	int edp_create_int();

	int move_to_synchro_position();
	int move_to_preset_position(int variant);

	int execute_motor_motion();
	int execute_joint_motion();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

