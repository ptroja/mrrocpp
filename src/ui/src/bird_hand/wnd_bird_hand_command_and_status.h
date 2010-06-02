// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_BIRD_HAND_COMMAND_AND_STATUS_H
#define __WND_BIRD_HAND_COMMAND_AND_STATUS_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury

class Ui;
class UiRobotBirdHand;

class WndBirdHandCommandAndStatus {
private:
	Ui& ui;
	UiRobotBirdHand& bird_hand;

public:
	bool is_open;

	WndBirdHandCommandAndStatus(Ui& _ui, UiRobotBirdHand& _bird_hand);

	int get_command();
	int set_status();
	int copy_command();

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

};

#endif

