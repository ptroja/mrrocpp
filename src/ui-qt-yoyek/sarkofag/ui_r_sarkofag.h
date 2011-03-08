// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SARKOFAG_H
#define __UI_R_SARKOFAG_H

#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/sarkofag/const_sarkofag.h"

class wgt_sarkofag_move;

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace tfg_and_conv {
class EcpRobot;
}
namespace sarkofag {

//
//
// KLASA UiRobotSarkofag
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot : public common::UiRobot
{
private:

public:

	double current_pos[lib::sarkofag::NUM_OF_SERVOS];// pozycja biezaca
	double desired_pos[lib::sarkofag::NUM_OF_SERVOS]; // pozycja zadana


	tfg_and_conv::EcpRobot *ui_ecp_robot;
	wgt_sarkofag_move *wgt_inc;

	UiRobot(common::Interface& _interface);

	int manage_interface();
	void delete_ui_ecp_robot();
	int synchronise();
	int synchronise_int();
	void edp_create();
	int edp_create_int();
	int execute_motor_motion();
	int execute_joint_motion();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

