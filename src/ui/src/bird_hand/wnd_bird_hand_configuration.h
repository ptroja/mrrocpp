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
namespace uin {
namespace common {
class Ui;
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
	common::Ui& ui;
	UiRobot& bird_hand;

public:
	bool is_open;

	WndConfiguration(common::Ui& _ui, UiRobot& _bird_hand);

	int get_configuration();
	int set_configuration();
	int copy_command();

};

}
} //namespace uin
} //namespace mrrocpp


#endif

