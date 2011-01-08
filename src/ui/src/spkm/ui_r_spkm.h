// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPKM_H
#define __UI_R_SPKM_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "robot/spkm/const_spkm.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm {

//
//
// KLASA UiRobot
//
//

class EcpRobot;
class WndInc;
class WndInt;
class WndExternal;

class UiRobot : public common::UiRobot
{
private:

public:

	double current_pos[lib::spkm::NUM_OF_SERVOS]; // pozycja biezaca
	double desired_pos[lib::spkm::NUM_OF_SERVOS]; // pozycja zadana


	EcpRobot *ui_ecp_robot;
	WndInc *wnd_inc;
	WndInt *wnd_int;
	WndExternal *wnd_external;

	UiRobot(common::Interface& _interface);

	int manage_interface();
	void delete_ui_ecp_robot();
	int synchronise();
	int synchronise_int();
	void edp_create();
	int edp_create_int();

	int move_to_synchro_position();
	int move_to_front_position();
	int move_to_preset_position(int variant);

	int execute_motor_motion();
	int execute_joint_motion();

};

}
} //namespace ui
} //namespace mrrocpp

#endif

