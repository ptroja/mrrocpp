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
namespace uin {
namespace common {

class Ui;

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury

class ui_bird_hand_robot;
class WndBirdHandCommandAndStatus;
class WndBirdHandConfiguration;

class UiRobotBirdHand: public UiRobot {
private:

public:
	ui_bird_hand_robot *ui_ecp_robot;
	WndBirdHandCommandAndStatus *wnd_command_and_status;
	WndBirdHandConfiguration *wnd_configuration;

	UiRobotBirdHand(Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int close_all_windows();
	int delete_ui_ecp_robot();
};

}
} //namespace uin
} //namespace mrrocpp

#endif

