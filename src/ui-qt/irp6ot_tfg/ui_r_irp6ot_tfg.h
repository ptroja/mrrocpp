// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6OT_TFG_H
#define __UI_R_IRP6OT_TFG_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_r_single_motor.h"
#include "../irp6_m/ui_r_irp6_m.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"


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
namespace irp6ot_tfg {

//
//
// KLASA UiRobot
//
//


class UiRobot : public single_motor::UiRobot
{
Q_OBJECT

public:
	UiRobot(common::Interface& _interface);

	void manage_interface();

	void synchronise();
	int synchronise_int();

	void move_to_synchro_position();
	void move_to_preset_position(int variant);

	int execute_motor_motion();
	int execute_joint_motion();

	void create_ui_ecp_robot();
	void edp_create_int_extra_operations();


	void setup_menubar();

private:
    QAction *actionirp6ot_tfg_Move;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

