// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SHEAD_H
#define __UI_R_SHEAD_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_robot.h"

namespace Ui {
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace shead {

class EcpRobot;
//
//
// KLASA UiRobot
//
//


class UiRobot : public common::UiRobot
{
Q_OBJECT

public:

	EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name);

	int manage_interface();
	void delete_ui_ecp_robot();
	void null_ui_ecp_robot();
	int synchronise();

	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);


	void setup_menubar();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

