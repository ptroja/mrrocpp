// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_CONVEYOR_H
#define __UI_R_CONVEYOR_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_r_single_motor.h"
#include "robot/conveyor/const_conveyor.h"

namespace Ui {
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

namespace conveyor {

//
// KLASA UiRobotConveyor
//
//

class UiRobot : public single_motor::UiRobot
{
Q_OBJECT
private:

public:

	UiRobot(common::Interface& _interface);

	void manage_interface();

	void synchronise();
	int synchronise_int();

	void create_ui_ecp_robot();
	void edp_create_int_extra_operations();



	void setup_menubar();

private:
	QAction *actionconveyor_Synchronization;
	QAction *actionconveyor_Move;
	QAction *actionconveyor_Synchro_Position;
	QAction *actionconveyor_Position_0;
	QAction *actionconveyor_Position_1;
	QAction *actionconveyor_Position_2;

	QMenu *menuconveyor_Preset_Positions;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

