// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_BIRD_HAND_H
#define __UI_R_BIRD_HAND_H

#include <QObject>
#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"

class wgt_bird_hand_command;
class WndConfiguration;


namespace Ui{
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace bird_hand {

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury

class EcpRobot;

class UiRobot : public common::UiRobot
{
Q_OBJECT

public:
	EcpRobot *ui_ecp_robot;


	UiRobot(common::Interface& _interface);

	int manage_interface();
	void delete_ui_ecp_robot();
	void null_ui_ecp_robot();
	int synchronise();
	void edp_create();
	void create_ui_ecp_robot();
	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);

	void setup_menubar();

	void make_connection();

	const static std::string WGT_COMMAND_AND_STATUS;
	const static std::string WGT_CONFIGURATION;

private:
	QAction *actionbirdhand_Command;
//	QAction *actionbirdhand_Configuration;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

