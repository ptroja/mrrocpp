// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SARKOFAG_H
#define __UI_R_SARKOFAG_H

#include "../base/ui.h"
#include "../base/ui_r_single_motor.h"
#include "robot/sarkofag/const_sarkofag.h"

class wgt_single_motor_move;

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace single_motor {
class EcpRobot;
}
namespace sarkofag {

//
//
// KLASA UiRobotSarkofag
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot : public single_motor::UiRobot
{
private:

public:

	UiRobot(common::Interface& _interface);

	int manage_interface();

	int synchronise();
	int synchronise_int();

	int execute_motor_motion();
	int execute_joint_motion();

	int create_ui_ecp_robot();
	int edp_create_int_extra_operations();

	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);

};

}
} //namespace ui
} //namespace mrrocpp

#endif

