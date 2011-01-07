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

	int manage_interface();
	void delete_ui_ecp_robot();
	int synchronise();
	void edp_create();
	int edp_create_int();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

