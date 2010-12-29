/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/wnd_base.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace common {

//
//
// KLASA WndBase
//
//


WndBase::WndBase(Interface& _interface, PtWidget_t * _ABW_window, ApEventLink_t *_ABM_window) :
	interface(_interface), is_open(false), ABW_window(_ABW_window), ABM_window(_ABM_window)
{

}

int WndBase::start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	if (!is_open) // otworz okno
	{
		ApCreateModule(ABM_window, widget, cbinfo);
		is_open = true;

	} else { // przelacz na okno
		PtWindowToFront(ABW_window);

	}

	return 1;
}

int WndBase::close()
{

	if (is_open) {
		PtDestroyWidget(ABW_window);
	}

	return 1;
}

int WndBase::clear_flag()
{

	is_open = false;

	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
