/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.03  */

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <strings.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <csignal>
#include <sys/netmgr.h>
#include <cerrno>
#include <process.h>
#include <cmath>

#include "base/lib/sr/srlib.h"

#include "ui/src/ui_class.h"
#include "ui/src/bird_hand/wnd_bird_hand_command_and_status.h"
#include "ui/src/bird_hand/wnd_bird_hand_configuration.h"
// #include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "robot/bird_hand/const_bird_hand.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

int EDP_bird_hand_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->edp_create();

	return (Pt_CONTINUE);

}

int EDP_bird_hand_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->EDP_slay_int();

	return (Pt_CONTINUE);

}

int execute_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_command_and_status->get_command();

	return (Pt_CONTINUE);
}

int copy_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_command_and_status->copy_command();

	return (Pt_CONTINUE);

}

int init_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_command_and_status->set_status();

	return (Pt_CONTINUE);

}

int start_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_command_and_status->start(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int clear_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_command_and_status->clear_flag();
	return (Pt_CONTINUE);

}

int execute_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_configuration->get_configuration();

	return (Pt_CONTINUE);
}

int copy_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_configuration->copy_command();

	return (Pt_CONTINUE);

}

int init_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_configuration->set_configuration();

	return (Pt_CONTINUE);

}

int start_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_configuration->start(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int clear_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.bird_hand->wnd_configuration->clear_flag();
	return (Pt_CONTINUE);

}
