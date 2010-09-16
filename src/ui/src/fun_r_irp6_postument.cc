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
#include <boost/bind.hpp>

#include "base/lib/sr/srlib.h"

#include "ui/src/ui_class.h"
// #include "ui/src/ui.h"
// Konfigurator.
// #include "base/lib/configurator.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"

#include "ui/src/ui_ecp_r_irp6_common.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

// zamykanie okien ruchow recznych dla robota irp6_postument


int close_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (interface.irp6p_m->is_wind_irp6p_inc_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_inc);
	}

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (interface.irp6p_m->is_wind_irp6p_int_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_int);
	}

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_xyz_angle_axis);
	}

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_xyz_euler_zyz);
	}

	return (Pt_CONTINUE);

}

int start_wnd_irp6_postument_xyz_aa_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_ts_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_xyz_angle_axis_ts, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_ts_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_xyz_angle_axis_ts);
	}

	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_xyz_angle_axis_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_ts_open = 0;

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_ts_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_xyz_angle_axis_ts);
	}

	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_kinematic_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_kinematic_open = 0;

	return (Pt_CONTINUE);

}

int start_wnd_irp6_postument_xyz_eul_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_ts_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_xyz_euler_zyz_ts, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_ts_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_xyz_euler_zyz_ts);
	}

	return (Pt_CONTINUE);

}

int start_wnd_irp6_postument_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_kinematic_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_kinematic, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_kinematic_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_kinematic);
	}

	return (Pt_CONTINUE);

}

int start_wnd_irp6_postument_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_servo_algorithm, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_xyz_euler_zyz_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_ts_open = 0;

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_ts_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_xyz_euler_zyz_ts);
	}

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->is_wind_irp6p_kinematic_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_kinematic);
	}

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->is_wind_irp6p_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int start_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_inc_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_inc, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_inc_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_inc);
	}

	return (Pt_CONTINUE);
}

int start_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_int_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_int, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_int_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_int);
	}

	return (Pt_CONTINUE);
}

int start_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_xyz_euler_zyz, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_xyz_euler_zyz);
	}

	return (Pt_CONTINUE);
}

int start_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_xyz_angle_axis, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_xyz_angle_axis);
	}

	return (Pt_CONTINUE);
}

int clear_wnd_irp6p_inc_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_inc_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_int_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_xyz_euler_zyz_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_xyz_euler_zyz_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_xyz_angle_axis_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_xyz_angle_axis_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}

int import_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char *tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p0, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p1, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p2, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p3, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p4, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p5, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p6, Pt_ARG_NUMERIC_VALUE,
			&val, 0);

	return (Pt_CONTINUE);

}

int export_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[7];

	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p0, Pt_ARG_NUMERIC_VALUE,
			&wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p1, Pt_ARG_NUMERIC_VALUE,
			&wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p2, Pt_ARG_NUMERIC_VALUE,
			&wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p3, Pt_ARG_NUMERIC_VALUE,
			&wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p4, Pt_ARG_NUMERIC_VALUE,
			&wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p5, Pt_ARG_NUMERIC_VALUE,
			&wektor[5], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p6, Pt_ARG_NUMERIC_VALUE,
			&wektor[6], 0);

	sprintf(buffer, "edp_IRP6_P INCREMENTAL POSITION\n %f %f %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5], *wektor[6]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);

}

int import_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p1, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p2, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p3, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p4, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p5, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p6, Pt_ARG_NUMERIC_VALUE,
			&val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p7, Pt_ARG_NUMERIC_VALUE,
			&val, 0);

	return (Pt_CONTINUE);

}

int export_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[7];

	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p1, Pt_ARG_NUMERIC_VALUE,
			&wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p2, Pt_ARG_NUMERIC_VALUE,
			&wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p3, Pt_ARG_NUMERIC_VALUE,
			&wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p4, Pt_ARG_NUMERIC_VALUE,
			&wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p5, Pt_ARG_NUMERIC_VALUE,
			&wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p6, Pt_ARG_NUMERIC_VALUE,
			&wektor[5], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p7, Pt_ARG_NUMERIC_VALUE,
			&wektor[6], 0);

	sprintf(buffer, "edp_IRP6_P INTERNAL POSITION\n %f %f %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5], *wektor[6]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);

}

int import_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p1,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p2,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p3,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p4,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p5,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p6,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p7,
			Pt_ARG_NUMERIC_VALUE, &val, 0);

	return (Pt_CONTINUE);

}

int export_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[7];

	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p1,
			Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p2,
			Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p3,
			Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p4,
			Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p5,
			Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p6,
			Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p7,
			Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

	sprintf(buffer, "edp_IRP6_P XYZ_EULER_ZYZ POSITION\n %f %f %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5], *wektor[6]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);

}

int init_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{

				interface.unblock_widget(ABW_PtPane_wind_irp6p_inc_post_synchro_moves);
				interface.irp6p_m->ui_ecp_robot->read_motors(interface.irp6p_m->irp6p_current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p0,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p1,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[1], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p2,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[2], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p3,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[3], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p4,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[4], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p5,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[5], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p6,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[6], 0);

				for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++)
					interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->irp6p_current_pos[i];
			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				interface.block_widget(ABW_PtPane_wind_irp6p_inc_post_synchro_moves);
			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wnd_irp6p_motors_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[lib::irp6p_m::NUM_OF_SERVOS], wektor[lib::irp6p_m::NUM_OF_SERVOS];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p0,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_motors_cur_p6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p0,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6p_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (interface.irp6p_m->state.edp.pid != -1) {

		if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_postument_preset_position_synchro) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_synchro)) || ((cbinfo->event->type == Ph_EV_KEY)
				&& (my_data->key_cap == 0x73))) && (interface.irp6p_m->state.edp.is_synchronised)) {// powrot do pozycji synchronizacji
			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = 0.0;
			}
			interface.irp6p_m->eb.command(boost::bind(irp6p_execute_motor_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_postument_preset_position_0) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_0)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x30))) && (interface.irp6p_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->state.edp.preset_position[0][i];
			}
			interface.irp6p_m->eb.command(boost::bind(irp6p_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_postument_preset_position_1) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_1)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x31))) && (interface.irp6p_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->state.edp.preset_position[1][i];
			}
			interface.irp6p_m->eb.command(boost::bind(irp6p_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_postument_preset_position_2) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_2)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x32))) && (interface.irp6p_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->state.edp.preset_position[2][i];
			}
			interface.irp6p_m->eb.command(boost::bind(irp6p_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_postument_preset_position_front)
				|| (ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_front)) || ((cbinfo->event->type
				== Ph_EV_KEY) && (my_data->key_cap == 0x66))) && (interface.irp6p_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->state.edp.front_position[i];
			}
			interface.irp6p_m->eb.command(boost::bind(irp6p_execute_joint_motion));
		}

	} // end if (interface.irp6p_m->state.edp.pid!=-1)


	return (Pt_CONTINUE);
}

int irp6p_execute_motor_motion()
{
	try {

		interface.irp6p_m->ui_ecp_robot->move_motors(interface.irp6p_m->irp6p_desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int irp6p_execute_joint_motion()
{
	try {

		interface.irp6p_m->ui_ecp_robot->move_joints(interface.irp6p_m->irp6p_desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int irp6p_inc_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor[7];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {

			if (interface.irp6p_m->state.edp.is_synchronised) {

				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p0,
						Pt_ARG_NUMERIC_VALUE, &(wektor[0]), 0);
				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p1,
						Pt_ARG_NUMERIC_VALUE, &(wektor[1]), 0);
				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p2,
						Pt_ARG_NUMERIC_VALUE, &(wektor[2]), 0);
				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p3,
						Pt_ARG_NUMERIC_VALUE, &(wektor[3]), 0);
				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p4,
						Pt_ARG_NUMERIC_VALUE, &(wektor[4]), 0);
				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p5,
						Pt_ARG_NUMERIC_VALUE, &(wektor[5]), 0);
				PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_p6,
						Pt_ARG_NUMERIC_VALUE, &(wektor[6]), 0);

				for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
					interface.irp6p_m->irp6p_desired_pos[i] = *wektor[i];
				}
			} else {
				for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
					interface.irp6p_m->irp6p_desired_pos[i] = 0.0;
				}
			}

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_inc_step,
					Pt_ARG_NUMERIC_VALUE, &step1, 0);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_0l) {
				interface.irp6p_m->irp6p_desired_pos[0] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_1l) {
				interface.irp6p_m->irp6p_desired_pos[1] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_2l) {
				interface.irp6p_m->irp6p_desired_pos[2] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_3l) {
				interface.irp6p_m->irp6p_desired_pos[3] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_4l) {
				interface.irp6p_m->irp6p_desired_pos[4] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_5l) {
				interface.irp6p_m->irp6p_desired_pos[5] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_6l) {
				interface.irp6p_m->irp6p_desired_pos[6] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_0r) {
				interface.irp6p_m->irp6p_desired_pos[0] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_1r) {
				interface.irp6p_m->irp6p_desired_pos[1] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_2r) {
				interface.irp6p_m->irp6p_desired_pos[2] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_3r) {
				interface.irp6p_m->irp6p_desired_pos[3] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_4r) {
				interface.irp6p_m->irp6p_desired_pos[4] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_5r) {
				interface.irp6p_m->irp6p_desired_pos[5] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_inc_6r) {
				interface.irp6p_m->irp6p_desired_pos[6] += (*step1);
			}

			interface.irp6p_m->ui_ecp_robot->move_motors(interface.irp6p_m->irp6p_desired_pos);

			if ((interface.irp6p_m->state.edp.is_synchronised) && (interface.irp6p_m->is_wind_irp6p_inc_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p0,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p1,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[1], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p2,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[2], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p3,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[3], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p4,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[4], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p5,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[5], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_inc_p6,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[6], 0);

			}
		} // end if (interface.irp6p_m->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int init_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_m->ui_ecp_robot->read_joints(interface.irp6p_m->irp6p_current_pos); // Odczyt polozenia walow silnikow

				// 	interface.unblock_widget(ABW_PtPane_wind_irp6p_int_post_synchro_moves);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p1,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p2,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[1], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p3,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[2], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p4,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[3], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p5,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[4], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p6,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[5], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p7,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[6], 0);

				for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++)
					interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->irp6p_current_pos[i];
			} else {
				// 	interface.block_widget(ABW_PtPane_wind_irp6p_int_post_synchro_moves);
			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int wnd_irp6p_joints_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[lib::irp6p_m::NUM_OF_SERVOS], wektor[lib::irp6p_m::NUM_OF_SERVOS];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_joints_cur_p7,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p7,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6p_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	double *wektor[lib::irp6p_m::NUM_OF_SERVOS];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((interface.irp6p_m->state.edp.pid != -1) && (interface.irp6p_m->state.edp.is_synchronised)) {

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_p7,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = *wektor[i];
			}

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_int_step,
					Pt_ARG_NUMERIC_VALUE, &step1, 0);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_1l) {
				interface.irp6p_m->irp6p_desired_pos[0] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_2l) {
				interface.irp6p_m->irp6p_desired_pos[1] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_3l) {
				interface.irp6p_m->irp6p_desired_pos[2] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_4l) {
				interface.irp6p_m->irp6p_desired_pos[3] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_5l) {
				interface.irp6p_m->irp6p_desired_pos[4] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_6l) {
				interface.irp6p_m->irp6p_desired_pos[5] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_7l) {
				interface.irp6p_m->irp6p_desired_pos[6] -= (*step1);
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_1r) {
				interface.irp6p_m->irp6p_desired_pos[0] += *step1;
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_2r) {
				interface.irp6p_m->irp6p_desired_pos[1] += *step1;
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_3r) {
				interface.irp6p_m->irp6p_desired_pos[2] += *step1;
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_4r) {
				interface.irp6p_m->irp6p_desired_pos[3] += *step1;
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_5r) {
				interface.irp6p_m->irp6p_desired_pos[4] += *step1;
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_6r) {
				interface.irp6p_m->irp6p_desired_pos[5] += *step1;
			} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_int_7r) {
				interface.irp6p_m->irp6p_desired_pos[6] += *step1;
			}

			interface.irp6p_m->ui_ecp_robot->move_joints(interface.irp6p_m->irp6p_desired_pos);

			if (interface.irp6p_m->is_wind_irp6p_int_open) // Czy robot jest zsynchronizowany?
			{

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p1,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p2,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[1], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p3,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[2], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p4,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[3], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p5,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[4], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p6,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[5], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_int_p7,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_desired_pos[6], 0);

			}

		} // end if (interface.irp6p_m->state.edp.pid!=-1)
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int init_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_m->ui_ecp_robot->read_xyz_euler_zyz(interface.irp6p_m->irp6p_current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p1,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[0], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p2,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[1], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p3,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[2], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p4,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[3], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p5,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[4], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p6,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[5], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p7,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[6], 0);

				for (int i = 0; i < 7; i++)
					interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->irp6p_current_pos[i];
			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int wnd_irp6p_xyz_zyz_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[7], wektor[7];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_read_p7,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < 7; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p7,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6p_xyz_euler_zyz_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor[7];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p7,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

			for (int i = 0; i < 7; i++)
				interface.irp6p_m->irp6p_desired_pos[i] = *wektor[i];

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_step,
					Pt_ARG_NUMERIC_VALUE, &step1, 0);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_1l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[0] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_2l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[1] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_3l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[2] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_4l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[3] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_5l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[4] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_6l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[5] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_7l) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[6] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_1r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[0] += *step1;
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_2r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[1] += *step1;
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_3r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[2] += *step1;
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_4r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[3] += *step1;
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_5r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[4] += *step1;
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_6r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[5] += *step1;
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_euler_zyz_7r) {
				// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
				interface.irp6p_m->irp6p_desired_pos[6] += *step1;
			}
			interface.irp6p_m->ui_ecp_robot->move_xyz_euler_zyz(interface.irp6p_m->irp6p_desired_pos);

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p4,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p5,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p6,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_p7,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6p_m->irp6p_desired_pos[6], 0);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int init_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	double alfa, kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_m->ui_ecp_robot->read_xyz_angle_axis(interface.irp6p_m->irp6p_current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p1,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[0], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p2,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[1], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p3,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[2], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p4,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[3], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p5,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[4], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p6,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[5], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p8,
						Pt_ARG_NUMERIC_VALUE,
						&interface.irp6p_m->irp6p_current_pos[6], 0);

				for (int i = 0; i < 7; i++) {
					interface.irp6p_m->irp6p_desired_pos[i] = interface.irp6p_m->irp6p_current_pos[i];
				}
			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int wnd_irp6p_xyz_aa_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[7], wektor[7];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_read_p8,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < 7; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p8,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6p_xyz_angle_axis_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {
			double *wektor_ptgr[7];
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p8,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[6], 0);

			double wektor[7];
			for (int i = 0; i < 7; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			double *krok;
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_step,
					Pt_ARG_NUMERIC_VALUE, &krok, 0);

			// wektor przesuniecia
			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_1l)
				wektor[0] -= (*krok);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_1r)
				wektor[0] += (*krok);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_2l)
				wektor[1] -= (*krok);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_2r)
				wektor[1] += (*krok);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_3l)
				wektor[2] -= (*krok);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_3r)
				wektor[2] += (*krok);

			// parametry wersora obrotu


			// kat obrotu i chwytak

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_8l)
				wektor[6] -= (*krok);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_angle_axis_8r)
				wektor[6] += (*krok);

			// sprawdzenie dlugosci wersora


			// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
			for (int i = 0; i < 7; i++) {
				interface.irp6p_m->irp6p_desired_pos[i] = wektor[i];

			}

			// zlecenie wykonania ruchu
			interface.irp6p_m->ui_ecp_robot->move_xyz_angle_axis(interface.irp6p_m->irp6p_desired_pos);

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p8,
					Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);
		}

	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int EDP_irp6_postument_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->eb.command(boost::bind(EDP_irp6_postument_synchronise_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_irp6_postument_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_postument

		if ((interface.irp6p_m->state.edp.state > 0) && (interface.irp6p_m->state.edp.is_synchronised == false)) {
			interface.irp6p_m->ui_ecp_robot->ecp->synchronise();
			interface.irp6p_m->state.edp.is_synchronised = interface.irp6p_m->ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_postument niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu

	interface.manage_interface();

	return (Pt_CONTINUE);

}

int init_wnd_irp6_postument_xyz_aa_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				lib::Xyz_Angle_Axis_vector tool_vector;

				interface.irp6p_m->ui_ecp_robot->read_tool_xyz_angle_axis(tool_vector); // Odczyt polozenia walow silnikow

				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p1,
						Pt_ARG_NUMERIC_VALUE, &tool_vector[0], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p2,
						Pt_ARG_NUMERIC_VALUE, &tool_vector[1], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p3,
						Pt_ARG_NUMERIC_VALUE, &tool_vector[2], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p4,
						Pt_ARG_NUMERIC_VALUE, &tool_vector[3], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p5,
						Pt_ARG_NUMERIC_VALUE, &tool_vector[4], 0);
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p6,
						Pt_ARG_NUMERIC_VALUE, &tool_vector[5], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wnd_irp6p_xyz_aa_ts_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[6], wektor[6];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_read_p6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);

			for (int i = 0; i < 6; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6p_xyz_angle_axis_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor_ptgr[6], wektor[6];
	double tool_vector[6];
	//	double wl;
	//	double l_eps = 0;
	//	double kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_ts_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0);

			for (int i = 0; i < 6; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
			for (int i = 0; i < 6; i++) {
				tool_vector[i] = wektor[i];

			}

			// zlecenie wykonania ruchu
			interface.irp6p_m->ui_ecp_robot->set_tool_xyz_angle_axis(tool_vector);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int init_wnd_irp6_postument_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	lib::Xyz_Euler_Zyz_vector tool_vector;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_m->ui_ecp_robot->read_tool_xyz_euler_zyz(tool_vector); // Odczyt polozenia walow silnikow

				double w;
				w = tool_vector[0];
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p1,
						Pt_ARG_NUMERIC_VALUE, &w, 0);
				w = tool_vector[1];
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p2,
						Pt_ARG_NUMERIC_VALUE, &w, 0);
				w = tool_vector[2];
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p3,
						Pt_ARG_NUMERIC_VALUE, &w, 0);
				w = tool_vector[3];
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p4,
						Pt_ARG_NUMERIC_VALUE, &w, 0);
				w = tool_vector[4];
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p5,
						Pt_ARG_NUMERIC_VALUE, &w, 0);
				w = tool_vector[5];
				PtSetResource(
						ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p6,
						Pt_ARG_NUMERIC_VALUE, &w, 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wnd_irp6p_xyz_zyz_ts_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[6], wektor[6];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(
					ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_read_p6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);

			for (int i = 0; i < 6; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6p_xyz_euler_zyz_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor[6];
	double tool_vector[6];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p1,
					Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p2,
					Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p3,
					Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p4,
					Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p5,
					Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_euler_zyz_ts_p6,
					Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

			for (int i = 0; i < 6; i++) {
				tool_vector[i] = *wektor[i];
			}

			// zlecenie wykonania ruchu
			interface.irp6p_m->ui_ecp_robot->set_tool_xyz_euler_zyz(tool_vector);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int init_wnd_irp6_postument_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t model_no;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_m->ui_ecp_robot->get_kinematic(&model_no); // Odczyt polozenia walow silnikow

				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_read_kinematic_model_no,
						Pt_ARG_NUMERIC_VALUE, model_no, 0);
			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int irp6p_kinematic_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *model_no_tmp;
	uint8_t model_no_output;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_kinematic_model_no,
					Pt_ARG_NUMERIC_VALUE, &model_no_tmp, 0);

			model_no_output = *model_no_tmp;

			// zlecenie wykonania ruchu
			interface.irp6p_m->ui_ecp_robot->set_kinematic(model_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int init_wnd_irp6_postument_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[lib::irp6p_m::NUM_OF_SERVOS];
	uint8_t servo_par_no[lib::irp6p_m::NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.pid != -1) {
			if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_m->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no); // Odczyt polozenia walow silnikow

				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_1,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_2,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[1], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_3,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[2], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_4,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[3], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_5,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[4], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_6,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[5], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_7,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[6], 0);

				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_1,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_2,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[1], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_3,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[2], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_4,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[3], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_5,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[4], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_6,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[5], 0);
				PtSetResource(
						ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_7,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[6], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wnd_irp6p_ser_alg_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	uint8_t *wektor_ptgr[lib::irp6p_m::NUM_OF_SERVOS], *wektor2_ptgr[lib::irp6p_m::NUM_OF_SERVOS],
			wektor[lib::irp6p_m::NUM_OF_SERVOS], wektor2[lib::irp6p_m::NUM_OF_SERVOS];

	if (interface.irp6p_m->state.edp.pid != -1) {
		if (interface.irp6p_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{

			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_1,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_2,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_3,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_4,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_5,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_6,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_alg_7,
					Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_1,
					Pt_ARG_NUMERIC_VALUE, wektor[0], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_2,
					Pt_ARG_NUMERIC_VALUE, wektor[1], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_3,
					Pt_ARG_NUMERIC_VALUE, wektor[2], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_4,
					Pt_ARG_NUMERIC_VALUE, wektor[3], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_5,
					Pt_ARG_NUMERIC_VALUE, wektor[4], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_6,
					Pt_ARG_NUMERIC_VALUE, wektor[5], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_7,
					Pt_ARG_NUMERIC_VALUE, wektor[6], 0);

			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_1,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[0]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_2,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[1]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_3,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[2]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_4,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[3]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_5,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[4]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_6,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[5]), 0);
			PtGetResource(
					ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_read_par_7,
					Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[6]), 0);

			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				wektor2[i] = *wektor2_ptgr[i];
			}

			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_1,
					Pt_ARG_NUMERIC_VALUE, wektor2[0], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_2,
					Pt_ARG_NUMERIC_VALUE, wektor2[1], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_3,
					Pt_ARG_NUMERIC_VALUE, wektor2[2], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_4,
					Pt_ARG_NUMERIC_VALUE, wektor2[3], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_5,
					Pt_ARG_NUMERIC_VALUE, wektor2[4], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_6,
					Pt_ARG_NUMERIC_VALUE, wektor2[5], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_7,
					Pt_ARG_NUMERIC_VALUE, wektor2[6], 0);

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie


		}
	}

	return (Pt_CONTINUE);

}

int irp6p_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[lib::irp6p_m::NUM_OF_SERVOS];
	uint8_t servo_alg_no_output[lib::irp6p_m::NUM_OF_SERVOS];
	uint8_t *servo_par_no_tmp[lib::irp6p_m::NUM_OF_SERVOS];
	uint8_t servo_par_no_output[lib::irp6p_m::NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_1,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_2,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[1], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_3,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[2], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_4,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[3], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_5,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[4], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_6,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[5], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_alg_7,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[6], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_1,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_2,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[1], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_3,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[2], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_4,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[3], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_5,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[4], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_6,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[5], 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_servo_algorithm_par_7,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[6], 0);

			for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			interface.irp6p_m->ui_ecp_robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int EDP_irp6_postument_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	EDP_irp6_postumentcreate_int(widget, apinfo, cbinfo);

	if (interface.irp6p_m->state.edp.state == 0) {
		interface.irp6p_m->create_thread();
		interface.irp6p_m->eb.command(boost::bind(EDP_irp6_postument_create_int, widget, apinfo, cbinfo));
	}
	return (Pt_CONTINUE);

}

int EDP_irp6_postument_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota irp6_postument
		if (interface.irp6p_m->state.edp.state == 0) {

			interface.irp6p_m->state.edp.state = 0;
			interface.irp6p_m->state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += interface.irp6p_m->state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += interface.irp6p_m->state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(interface.irp6p_m->state.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0))
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "edp_irp6_postument already exists");
			} else if (interface.check_node_existence(interface.irp6p_m->state.edp.node_name, std::string("edp_irp6_postument"))) {
				interface.irp6p_m->state.edp.node_nr = interface.config->return_node_number(interface.irp6p_m->state.edp.node_name);

				{
					boost::unique_lock <boost::mutex> lock(interface.process_creation_mtx);

					interface.irp6p_m->ui_ecp_robot
							= new ui::irp6::EcpRobot(*interface.config, *interface.all_ecp_msg, lib::irp6p_m::ROBOT_NAME);
				}

				interface.irp6p_m->state.edp.pid = interface.irp6p_m->ui_ecp_robot->ecp->get_EDP_pid();

				if (interface.irp6p_m->state.edp.pid < 0) {
					interface.irp6p_m->state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete interface.irp6p_m->ui_ecp_robot;
				} else { // jesli spawn sie powiodl
					interface.irp6p_m->state.edp.state = 1;
					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia
					while ((interface.irp6p_m->state.edp.reader_fd
							= name_open(interface.irp6p_m->state.edp.network_reader_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL))
							< 0)
						if ((tmp++) < lib::CONNECT_RETRY) {
							delay(lib::CONNECT_DELAY);
						} else {
							perror("blad odwolania do READER_P");
							break;
						}
					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;
					interface.irp6p_m->ui_ecp_robot->get_controller_state(robot_controller_initial_state_tmp);

					//interface.irp6p_m->state.edp.state = 1; // edp wlaczone reader czeka na start
					interface.irp6p_m->state.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try
	CATCH_SECTION_UI

	interface.manage_interface();

	return 1;

}

int EDP_irp6_postument_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	EDP_irp6_postumentcreate_int(widget, apinfo, cbinfo);


	interface.irp6p_m->EDP_slay_int();

	return (Pt_CONTINUE);

}

int pulse_reader_irp6p_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->pulse_reader_start_exec_pulse()) {
		process_control_window_init(widget, apinfo, cbinfo);
	}

	return (Pt_CONTINUE);

}

int pulse_reader_irp6p_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->pulse_reader_stop_exec_pulse()) {
		process_control_window_init(widget, apinfo, cbinfo);
	}

	return (Pt_CONTINUE);

}

int pulse_reader_irp6p_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->pulse_reader_trigger_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_ecp_irp6_postument(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_m->state.edp.is_synchronised) { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (interface.irp6p_m->state.ecp.trigger_fd < 0) {

			short tmp = 0;
			// kilka sekund  (~1) na otworzenie urzadzenia
			// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem

			ualarm(ui::common::SIGALRM_TIMEOUT, 0);
			while ((interface.irp6p_m->state.ecp.trigger_fd
					= name_open(interface.irp6p_m->state.ecp.network_trigger_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL))
					< 0) {
				if (errno == EINTR)
					break;
				if ((tmp++) < lib::CONNECT_RETRY) {
					delay(lib::CONNECT_DELAY);
				} else {
					perror("blad odwolania do ECP_TRIGGER");
				};
			}
			// odwolanie alarmu
			ualarm((useconds_t)(0), 0);
		}

		if (interface.irp6p_m->state.ecp.trigger_fd >= 0) {
			if (MsgSendPulse(interface.irp6p_m->state.ecp.trigger_fd, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value)
					== -1) {

				fprintf(stderr, "Blad w wysylaniu pulsu do ecp error: %s \n", strerror(errno));
				delay(1000);
			}
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return (Pt_CONTINUE);

}

int start_wnd_irp6_postument_xyz_aa_relative(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_m->is_wind_irp6p_xyz_aa_relative_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_postument_xyz_angle_axis_relative, widget, cbinfo);
		interface.irp6p_m->is_wind_irp6p_xyz_aa_relative_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_postument_xyz_angle_axis_relative);
	}

	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_xyz_aa_relative_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_m->is_wind_irp6p_xyz_aa_relative_open = false;

	return (Pt_CONTINUE);

}

int close_wnd_irp6_postument_xyz_aa_relative(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (interface.irp6ot_m->is_wind_irp6ot_xyz_aa_relative_open) {
		PtDestroyWidget(ABW_wnd_irp6_postument_xyz_angle_axis_relative);
	}

	return (Pt_CONTINUE);

}

int irp6p_xyz_aa_relative_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor_ptgr[7], wektor[7];

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_px,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_py,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_pz,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_pox,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_poy,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_poz,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_aa_relative_pg,
					Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[6], 0);

			for (int i = 0; i < 7; i++) {
				wektor[i] = *wektor_ptgr[i];
				interface.irp6p_m->irp6p_desired_pos[i] = 0.0;
			}

			// wektor przesuniecia
			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_xl)
				interface.irp6p_m->irp6p_desired_pos[0] = -wektor[0];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_xr)
				interface.irp6p_m->irp6p_desired_pos[0] = wektor[0];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_yl)
				interface.irp6p_m->irp6p_desired_pos[1] = -wektor[1];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_yr)
				interface.irp6p_m->irp6p_desired_pos[1] = wektor[1];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_zl)
				interface.irp6p_m->irp6p_desired_pos[2] = -wektor[2];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_zr)
				interface.irp6p_m->irp6p_desired_pos[2] = wektor[2];

			// kat obrotu i chwytak
			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_oxl)
				interface.irp6p_m->irp6p_desired_pos[3] = -wektor[3];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_oxr)
				interface.irp6p_m->irp6p_desired_pos[3] = wektor[3];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_oyl)
				interface.irp6p_m->irp6p_desired_pos[4] = -wektor[4];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_oyr)
				interface.irp6p_m->irp6p_desired_pos[4] = wektor[4];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_ozl)
				interface.irp6p_m->irp6p_desired_pos[5] = -wektor[5];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_ozr)
				interface.irp6p_m->irp6p_desired_pos[5] = wektor[5];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_gl)
				interface.irp6p_m->irp6p_desired_pos[6] = -wektor[6];

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_gr)
				interface.irp6p_m->irp6p_desired_pos[6] = wektor[6];

			// wszysktkie naraz
			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_l)
				for (int i = 0; i < 7; i++) {
					interface.irp6p_m->irp6p_desired_pos[i] = -wektor[i];
				}

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6p_xyz_aa_r)
				for (int i = 0; i < 7; i++) {
					interface.irp6p_m->irp6p_desired_pos[i] = wektor[i];
				}

			// zlecenie wykonania ruchu
			interface.irp6p_m->ui_ecp_robot->move_xyz_angle_axis_relative(interface.irp6p_m->irp6p_desired_pos);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int import_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p1,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p2,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p3,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p4,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p5,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p6,
			Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p8,
			Pt_ARG_NUMERIC_VALUE, &val, 0);

	return (Pt_CONTINUE);

}

int export_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[7];

	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p1,
			Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p2,
			Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p3,
			Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p4,
			Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p5,
			Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p6,
			Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6p_xyz_angle_axis_p8,
			Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

	sprintf(buffer, "edp_IRP6_P XYZ_ANGLE_AXIS POSITION\n %f %f %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5], *wektor[6]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);

}

