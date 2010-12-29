/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/spkm/ui_r_spkm.h"
#include "ui/src/spkm/wnd_spkm_inc.h"
#include "robot/spkm/const_spkm.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

//
//
// KLASA WndInc
//
//


WndInc::WndInc(common::Interface& _interface, UiRobot& _robot) :
	common::WndBase(_interface), robot(_robot)
{

}

int WndInc::close()
{

	if (is_open) {
		PtDestroyWidget(ABW_wnd_spkm_inc);
	}

	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
