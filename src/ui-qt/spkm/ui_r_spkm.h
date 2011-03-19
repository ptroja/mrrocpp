// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPKM_H
#define __UI_R_SPKM_H

#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/spkm/const_spkm.h"
#include "robot/spkm/kinematic_parameters_spkm.h"

class wgt_spkm_inc;
class wgt_spkm_int;
class wgt_spkm_ext;

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

class UiRobot : public common::UiRobot
{
private:

public:

	double current_pos[lib::spkm::NUM_OF_SERVOS]; // pozycja biezaca
	double desired_pos[lib::spkm::NUM_OF_SERVOS]; // pozycja zadana

	kinematics::spkm::kinematic_parameters_spkm kinematic_params;

	EcpRobot *ui_ecp_robot;
	wgt_spkm_inc *wgt_inc;
	wgt_spkm_int *wgt_int;
	wgt_spkm_ext *wgt_ext;

	/* TR
	 WndInt *wnd_int;
	 WndExternal *wnd_external;
	 */

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

	int execute_clear_fault();
	int execute_stop_motor();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

