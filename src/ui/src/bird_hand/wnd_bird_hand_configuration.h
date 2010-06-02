// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_BIRD_HAND_CONFIGURATION_H
#define __WND_BIRD_HAND_CONFIGURATION_H

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

class WndBirdHandConfiguration {
private:
	Ui& ui;
	UiRobotBirdHand& bird_hand;

public:
	bool is_open;

	WndBirdHandConfiguration(Ui& _ui, UiRobotBirdHand& _bird_hand);

	int get_configuration();
	int set_configuration();
	int copy_command();

};

#endif

