// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_SPKM_INT_H
#define __WND_SPKM_INT_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "ui/src/wnd_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace spkm {

const std::string WND_SPKM_INT = "WND_SPKM_INT";

//
//
// KLASA WndInt
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot;

class WndInt : public common::WndBase
{
private:
	UiRobot& robot;

public:

	int init();
	int import();
	int exporto();
	int copy();
	int motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

	int set_single_axis(int axis, PtWidget_t *ABW_current, PtWidget_t *ABW_position, PtWidget_t *ABW_thumb);

	WndInt(common::Interface& _interface, UiRobot& _robot);

};

}
} //namespace ui
} //namespace mrrocpp


#endif

