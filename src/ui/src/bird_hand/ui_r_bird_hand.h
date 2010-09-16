// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_BIRD_HAND_H
#define __UI_R_BIRD_HAND_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

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
class WndCommandAndStatus;
class WndConfiguration;

class UiRobot : public common::UiRobot
{
private:

public:
	EcpRobot *ui_ecp_robot;
	WndCommandAndStatus *wnd_command_and_status;
	WndConfiguration *wnd_configuration;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();
	int close_all_windows();
	int delete_ui_ecp_robot();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

