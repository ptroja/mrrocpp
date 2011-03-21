// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_BIRD_HAND_H
#define __UI_R_BIRD_HAND_H

#include "../base/ui.h"
#include "../base/ui_robot.h"

class wgt_bird_hand_command;
class WndConfiguration;

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
private:

public:
	EcpRobot *ui_ecp_robot;
	wgt_bird_hand_command *wgt_command_and_status;
	WndConfiguration *wgt_configuration;



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

