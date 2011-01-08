/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

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
#include "ui/src/spkm/wnd_spkm_inc.h"
#include "ui/src/spkm/wnd_spkm_int.h"
#include "ui/src/spkm/wnd_spkm_external.h"

// #include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"

#include "robot/spkm/const_spkm.h"
/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

int EDP_spkm_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->edp_create();

	return (Pt_CONTINUE);

}

int EDP_spkm_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->EDP_slay_int();
	return (Pt_CONTINUE);

}

int EDP_spkm_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->synchronise();

	return (Pt_CONTINUE);

}

int start_wnd_spkm_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->start(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int init_wnd_spkm_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->init();

	return (Pt_CONTINUE);

}

int wnd_spkm_motors_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->copy();

	return (Pt_CONTINUE);

}

int clear_wnd_spkm_inc_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->clear_flag();

	return (Pt_CONTINUE);

}

int spkm_inc_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->motion(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int import_wnd_spkm_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->import();

	return (Pt_CONTINUE);

}

int export_wnd_spkm_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_inc->exporto();

	return (Pt_CONTINUE);

}

int init_wnd_spkm_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->init();

	return (Pt_CONTINUE);

}

int wnd_spkm_int_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->copy();

	return (Pt_CONTINUE);

}

int clear_wnd_spkm_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->clear_flag();

	return (Pt_CONTINUE);

}

int spkm_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->motion(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int import_wnd_spkm_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->import();

	return (Pt_CONTINUE);

}

int export_wnd_spkm_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->exporto();

	return (Pt_CONTINUE);

}

int start_wnd_spkm_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_int->start(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int start_wnd_spkm_external(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->start(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int init_wnd_spkm_external(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->init();

	return (Pt_CONTINUE);

}

int clear_wnd_spkm_external_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->clear_flag();

	return (Pt_CONTINUE);

}

int wnd_spkm_external_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->copy();

	return (Pt_CONTINUE);

}

int spkm_external_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->motion(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int import_wnd_spkm_external(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->import();

	return (Pt_CONTINUE);

}

int export_wnd_spkm_external(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.spkm->wnd_external->exporto();

	return (Pt_CONTINUE);

}

int spkm_move_to_synchro_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.spkm->move_to_synchro_position();
	return (Pt_CONTINUE);

}

int spkm_move_to_front_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.spkm->move_to_front_position();
	return (Pt_CONTINUE);

}

int spkm_move_to_preset_position_0(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.spkm->move_to_preset_position(0);
	return (Pt_CONTINUE);

}

int spkm_move_to_preset_position_1(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.spkm->move_to_preset_position(1);
	return (Pt_CONTINUE);

}

int spkm_move_to_preset_position_2(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.spkm->move_to_preset_position(2);
	return (Pt_CONTINUE);

}

