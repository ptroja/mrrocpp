// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_BIRD_HAND_CONFIGURATION_H
#define __WND_BIRD_HAND_CONFIGURATION_H

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


class UiRobot;

class WndConfiguration
{
private:
	common::Interface& interface;
	UiRobot& bird_hand;

public:
	bool is_open;

	WndConfiguration(common::Interface& _interface, UiRobot& _bird_hand);

	int get_configuration();
	int set_configuration();
	int copy_command();

};

}
} //namespace ui
} //namespace mrrocpp


#endif

