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
	const std::string window_name;
	common::Interface& interface;
	bool is_open;

	int ABN_window, ABI_window;

	int start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
	int close();
	int clear_flag();
	PtWidget_t * ABW_window();
	ApEventLink_t *ABM_window();

	WndBase(const std::string _window_name, common::Interface& _interface, int _ABN_window, int _ABI_window);

};

}
} //namespace ui
} //namespace mrrocpp


#endif

