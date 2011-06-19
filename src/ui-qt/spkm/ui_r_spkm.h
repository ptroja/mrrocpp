// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPKM_H
#define __UI_R_SPKM_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/spkm/const_spkm.h"
#include "robot/spkm/kinematic_parameters_spkm.h"

namespace Ui {
class MenuBar;
class MenuBarAction;
}

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
Q_OBJECT

public:

	double current_pos[lib::spkm::NUM_OF_SERVOS]; // pozycja biezaca
	double desired_pos[lib::spkm::NUM_OF_SERVOS]; // pozycja zadana

	kinematics::spkm::kinematic_parameters_spkm kinematic_params;

	EcpRobot *ui_ecp_robot;

	/* TR
	 WndInt *wnd_int;
	 WndExternal *wnd_external;
	 */

	UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name);

	int manage_interface();
	void delete_ui_ecp_robot();
	void null_ui_ecp_robot();
	int synchronise();
	int synchronise_int();

	int move_to_synchro_position();
	int move_to_front_position();
	int move_to_preset_position(int variant);

	int execute_motor_motion();
	int execute_joint_motion();

	int execute_clear_fault();
	int execute_stop_motor();
	int edp_create_int_extra_operations();
	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);

	void make_connections();
	void setup_menubar();

private:
	QAction *actionspkm_Synchronisation;
	QAction *actionspkm_Motors;
	QAction *actionspkm_Motors_post;
	QAction *actionspkm_Joints;
	QAction *actionspkm_External;
	QAction *actionspkm_Synchro_Position;
	QAction *actionspkm_Front_Position;
	QAction *actionspkm_Position_0;
	QAction *actionspkm_Position_1;
	QAction *actionspkm_Position_2;
	QAction *actionspkm_Clear_Fault;

	QMenu *menuspkm_Pre_synchro_moves;
	QMenu *menuspkm_Post_synchro_moves;
	QMenu *menuspkm_Preset_positions;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

