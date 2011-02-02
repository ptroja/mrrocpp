// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_BIRD_HAND_COMMAND_AND_STATUS_H
#define __WND_BIRD_HAND_COMMAND_AND_STATUS_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "ui/src/wnd_base.h"
#include "robot/bird_hand/const_bird_hand.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace bird_hand {
const std::string WND_BIRD_HAND_COMMAND_AND_STATUS = "WND_BIRD_HAND_COMMAND_AND_STATUS";
//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot;

class WndCommandAndStatus : public common::WndBase
{
private:
	UiRobot& bird_hand;

public:

	WndCommandAndStatus(common::Interface& _interface, UiRobot& _bird_hand);

	int get_command();
	int set_status();
	int copy_command();

	int
			get_variant_finger_command(lib::bird_hand::single_joint_command &finger, PtWidget_t *ABW_absolute, PtWidget_t *ABW_relative, PtWidget_t *ABW_velocity);

	int
			get_finger_command(lib::bird_hand::single_joint_command &finger, PtWidget_t *ABW_position, PtWidget_t *ABW_torque, PtWidget_t *ABW_damping);
	int
			set_finger_status(lib::bird_hand::single_joint_status &finger, PtWidget_t *ABW_position, PtWidget_t *ABW_torque, PtWidget_t *ABW_current, PtWidget_t *ABW_limit_1, PtWidget_t *ABW_limit_2, PtWidget_t *ABW_limit_3, PtWidget_t *ABW_limit_4, PtWidget_t *ABW_limit_5, PtWidget_t *ABW_limit_6, PtWidget_t *ABW_limit_7, PtWidget_t *ABW_limit_8);

	int
	copy_finger_command(lib::bird_hand::single_joint_command &finger, PtWidget_t *ABW_current, PtWidget_t *ABW_desired);

};

}
} //namespace ui
} //namespace mrrocpp

#endif

