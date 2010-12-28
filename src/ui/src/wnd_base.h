// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_BASE_H
#define __WND_BASE_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury


class WndBase
{
private:

public:
	common::Interface& interface;
	bool is_open;

	virtual int close() = 0;

	WndBase(common::Interface& _interface);

};

}
} //namespace ui
} //namespace mrrocpp


#endif

