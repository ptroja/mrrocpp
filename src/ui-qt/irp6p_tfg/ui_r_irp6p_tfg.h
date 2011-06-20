// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6P_TFG_H
#define __UI_R_IRP6P_TFG_H

#include <QObject>
#include "../base/mainwindow.h"
#include "../base/interface.h"
#include "../base/ui.h"
#include "../base/ui_r_single_motor.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"

namespace Ui{
class MenuBar;
class MenuBarAction;
}

namespace Ui{
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace single_motor {
class EcpRobot;
}
namespace irp6p_tfg {

//
//
// KLASA UiRobot
//
//


class UiRobot : public single_motor::UiRobot
{
	Q_OBJECT
private:

public:

	UiRobot(common::Interface& _interface);

	int manage_interface();

	int synchronise();
	int synchronise_int();

	int move_to_synchro_position();
	int move_to_preset_position(int variant);

	int execute_motor_motion();
	int execute_joint_motion();

	int create_ui_ecp_robot();
	int edp_create_int_extra_operations();

	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);
	void make_connections();
	void setup_menubar();

private:
	QAction *actionirp6p_tfg_Synchronization;
	QAction *actionirp6p_tfg_Move;
	QAction *actionirp6p_tfg_Synchro_Position;
	QAction *actionirp6p_tfg_Position_0;
	QAction *actionirp6p_tfg_Position_1;
	QAction *actionirp6p_tfg_Position_2;

    QMenu *menuirp6p_tfg_Preset_Positions;
};

}
} //namespace ui
} //namespace mrrocpp

#endif

