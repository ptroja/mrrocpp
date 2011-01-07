// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __WND_SPKM_INC_H
#define __WND_SPKM_INC_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "ui/src/wnd_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace spkm {

const std::string WND_SPKM_INC = "WND_SPKM_INC";



//
//
// KLASA WndInc
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot;

class WndInc : public common::WndBase
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

	WndInc(common::Interface& _interface, UiRobot& _robot);

};

}
} //namespace ui
} //namespace mrrocpp


#endif

