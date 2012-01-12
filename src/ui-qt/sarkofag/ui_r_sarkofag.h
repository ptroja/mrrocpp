// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SARKOFAG_H
#define __UI_R_SARKOFAG_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_r_single_motor.h"
#include "robot/sarkofag/const_sarkofag.h"

namespace Ui{
class MenuBar;
class MenuBarAction;
}

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
	Q_OBJECT

public:

	UiRobot(common::Interface& _interface);

	void manage_interface();

	void synchronise();
	int synchronise_int();

	int execute_motor_motion();
	int execute_joint_motion();

	void create_ui_ecp_robot();
	void edp_create_int_extra_operations();

	void setup_menubar();

private:
    QAction *actionsarkofag_Synchronisation;
    QAction *actionsarkofag_Move;
    QAction *actionsarkofag_Synchro_Position;
    QAction *actionsarkofag_Front_Position;
    QAction *actionsarkofag_Position_0;
    QAction *actionsarkofag_Position_1;
    QAction *actionsarkofag_Position_2;
    QAction *actionsarkofag_Servo_Algorithm;

    QMenu *menusarkofag_Preset_Positions;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

