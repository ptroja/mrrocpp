// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6P_M_H
#define __UI_R_IRP6P_M_H

#include "../base/ui.h"
#include "../irp6_m/ui_r_irp6_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include <QObject>
#include <QMenu>

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"

namespace Ui {
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class EcpRobot;
}

namespace irp6p_m {

//
//
// KLASA UiRobotIrp6p_m
//
//

class UiRobot : public irp6_m::UiRobot
{
Q_OBJECT

public:

	UiRobot(common::Interface& _interface);

	void synchronise();

	/*
	 * opens move window on mp or ecp request
	 * 	 * C_MOTOR variant
	 */

	void open_c_motor_window();

	/*
	 * opens move window on mp or ecp request
	 * 	 * C_JOINT variant
	 */

	void open_c_joint_window();

	void move_to_synchro_position();
	void move_to_front_position();
	void move_to_preset_position(int variant);
	void create_ui_ecp_robot();
	void edp_create_int_extra_operations();

	void setup_menubar();

private:
	QAction *actionirp6p_m_Motors;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

