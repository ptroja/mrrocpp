// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6OT_TFG_H
#define __UI_R_IRP6OT_TFG_H

#include "../base/ui.h"
#include "../base/ui_r_single_motor.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace single_motor {
class EcpRobot;
}
namespace irp6ot_tfg {

//
//
// KLASA UiRobot
//
//


class UiRobot : public single_motor::UiRobot
{
private:

public:

	UiRobot(common::Interface& _interface);

	int manage_interface();

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

