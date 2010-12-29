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
#include "ablibs.h"

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
	PtWidget_t * ABW_window;

	int close();

	WndBase(common::Interface& _interface, PtWidget_t * _ABW_window);

};

}
} //namespace ui
} //namespace mrrocpp


#endif

