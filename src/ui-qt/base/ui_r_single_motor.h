// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SINGLE_MOTOR_H
#define __UI_R_SINGLE_MOTOR_H

#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/conveyor/const_conveyor.h"

class wgt_single_motor_move;

namespace Ui{
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class EcpRobot;
}
namespace single_motor {

class UiRobot : public common::UiRobot
{
private:

public:

//	double current_pos[1];// pozycja biezaca
//	double desired_pos[1]; // pozycja zadana


	common::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos);
	void delete_ui_ecp_robot();
	void null_ui_ecp_robot();

	void setup_menubar();
	int manage_interface();

protected:
	QAction *action_Synchronisation;

private:
    QAction *action_Synchro_Position;
    QAction *action_Front_Position;
    QAction *action_Position_0;
    QAction *action_Position_1;
    QAction *action_Position_2;

    QMenu *menu_Preset_Positions;
};

}
} //namespace ui
} //namespace mrrocpp

#endif

