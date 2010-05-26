// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_CLASS_H
#define __UI_CLASS_H

#include "ui/ui.h"
#include "ui/ui_r_bird_hand.h"
#include "lib/configurator.h"

//
//
// KLASA ui
//
//


// super klasa agregujaca porozrzucane struktury


class Ui {
private:

public:
	lib::configurator* config;
	UiRobotBirdHand bird_hand;

	Ui();

};

#endif

