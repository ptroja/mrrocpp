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
#include "ui/src/wnd_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace bird_hand {

const std::string WND_BIRD_HAND_CONFIGURATION = "WND_BIRD_HAND_CONFIGURATION";

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot;

class WndConfiguration : public common::WndBase
{
private:
	UiRobot& bird_hand;

public:

	WndConfiguration(common::Interface& _interface, UiRobot& _bird_hand);

	int get_configuration();
	int set_configuration();
	int copy_command();

};

}
} //namespace ui
} //namespace mrrocpp


#endif

