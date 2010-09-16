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

class WndCommandAndStatus
{
private:
	common::Interface& interface;
	UiRobot& bird_hand;

public:
	bool is_open;

	WndCommandAndStatus(common::Interface& _interface, UiRobot& _bird_hand);

	int get_command();
	int set_status();
	int copy_command();

	//
	//
	// thumb_f_0
	//
	//

	int get_thumb_f_0_command();
	int get_variant_thumb_f_0_command();
	int set_thumb_f_0_status();
	int copy_thumb_f_0_command();

	//
	//
	// thumb_f_1
	//
	//

	int get_thumb_f_1_command();
	int get_variant_thumb_f_1_command();
	int set_thumb_f_1_status();
	int copy_thumb_f_1_command();

	//
	//
	// index_f_0
	//
	//

	int get_index_f_0_command();
	int get_variant_index_f_0_command();
	int set_index_f_0_status();
	int copy_index_f_0_command();

	//
	//
	// index_f_1
	//
	//

	int get_index_f_1_command();
	int get_variant_index_f_1_command();
	int set_index_f_1_status();
	int copy_index_f_1_command();

	//
	//
	// index_f_2
	//
	//

	int get_index_f_2_command();
	int get_variant_index_f_2_command();
	int set_index_f_2_status();
	int copy_index_f_2_command();

	//
	//
	// ring_f_0
	//
	//

	int get_ring_f_0_command();
	int get_variant_ring_f_0_command();
	int set_ring_f_0_status();
	int copy_ring_f_0_command();

	//
	//
	// ring_f_1
	//
	//

	int get_ring_f_1_command();
	int get_variant_ring_f_1_command();
	int set_ring_f_1_status();
	int copy_ring_f_1_command();

	//
	//
	// ring_f_2
	//
	//

	int get_ring_f_2_command();
	int get_variant_ring_f_2_command();
	int set_ring_f_2_status();
	int copy_ring_f_2_command();

};

}
} //namespace ui
} //namespace mrrocpp

#endif

