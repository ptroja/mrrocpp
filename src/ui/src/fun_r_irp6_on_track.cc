/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <semaphore.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <errno.h>
#include <process.h>
#include <math.h>

#include "lib/srlib.h"
#include "ui/ui_const.h"
// #include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"


extern ui_msg_def ui_msg;
extern ui_ecp_buffer* ui_ecp_obj;

extern ui_state_def ui_state;
extern configurator* config;

extern ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;

double irp6ot_current_pos[8]; // pozycja biezaca
double irp6ot_desired_pos[8]; // pozycja zadana

// zamykanie okien ruchow recznych dla robota irp6_on_track


int
close_wnd_irp6_on_track_inc( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (ui_state.is_wind_irp6ot_inc_open)
	{
		PtDestroyWidget( ABW_wnd_irp6_on_track_inc );
	}

	return( Pt_CONTINUE );

	}


int
close_wnd_irp6_on_track_int( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (ui_state.is_wind_irp6ot_int_open)
	{
		PtDestroyWidget( ABW_wnd_irp6_on_track_int );
	}

	return( Pt_CONTINUE );

	}


int
close_wnd_irp6_on_track_xyz_angle_axis( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6ot_xyz_angle_axis_open)
	{
		PtDestroyWidget( ABW_wnd_irp6_on_track_xyz_angle_axis );
	}

	return( Pt_CONTINUE );

	}


int
close_wnd_irp6_on_track_xyz_euler_zyz( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6ot_xyz_euler_zyz_open)
	{
		PtDestroyWidget( ABW_wnd_irp6_on_track_xyz_euler_zyz );
	}

	return( Pt_CONTINUE );

	}




int
close_wnd_irp6_on_track_xyz_angle_axis_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open)
	{
		PtDestroyWidget ( ABW_wnd_irp6_on_track_xyz_angle_axis_ts );
	}

	return( Pt_CONTINUE );

	}


int
clear_wnd_irp6ot_xyz_angle_axis_ts_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open=0;

	return( Pt_CONTINUE );

	}


int
clear_wnd_irp6ot_kinematic_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_kinematic_open = 0;

	return( Pt_CONTINUE );

	}



int
start_wnd_irp6ot_xyz_angle_axis_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_xyz_angle_axis_ts, widget, cbinfo);
		ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_xyz_angle_axis_ts);
	}


	return( Pt_CONTINUE );

	}



int
start_wnd_irp6_on_track_xyz_euler_zyz_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_xyz_euler_zyz_ts, widget, cbinfo);
		ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_xyz_euler_zyz_ts);
	}


	return( Pt_CONTINUE );

	}




int
start_wnd_irp6_on_track_kinematic( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_kinematic_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_kinematic, widget, cbinfo);
		ui_state.is_wind_irp6ot_kinematic_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_kinematic);
	}

	return( Pt_CONTINUE );

	}



int
start_wnd_irp6_on_track_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_servo_algorithm_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_servo_algorithm, widget, cbinfo);
		ui_state.is_wind_irp6ot_servo_algorithm_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_servo_algorithm);
	}

	return( Pt_CONTINUE );

	}


int
clear_wnd_irp6ot_xyz_euler_zyz_ts_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open = 0;

	return( Pt_CONTINUE );

	}


int
close_wnd_irp6_on_track_xyz_euler_zyz_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open)
	{
		PtDestroyWidget ( ABW_wnd_irp6_on_track_xyz_euler_zyz_ts );
	}

	return( Pt_CONTINUE );

	}



int
close_wnd_irp6_on_track_kinematic( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6ot_kinematic_open)
	{
		PtDestroyWidget ( ABW_wnd_irp6_on_track_kinematic );
	}

	return( Pt_CONTINUE );

	}


int
close_wnd_irp6_on_track_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6ot_servo_algorithm_open)
	{
		PtDestroyWidget ( ABW_wnd_irp6_on_track_servo_algorithm );
	}

	return( Pt_CONTINUE );

	}




int
start_wnd_irp6_on_track_inc( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_inc_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_inc, widget, cbinfo);
		ui_state.is_wind_irp6ot_inc_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_inc);
	}

	return( Pt_CONTINUE );
}


int
start_wnd_irp6_on_track_int( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_int_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_int, widget, cbinfo);
		ui_state.is_wind_irp6ot_int_open=true;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_int);
	}

	return( Pt_CONTINUE );
}


int
start_wnd_irp6_on_track_xyz_euler_zyz( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_xyz_euler_zyz_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_xyz_euler_zyz, widget, cbinfo);
		ui_state.is_wind_irp6ot_xyz_euler_zyz_open=true;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_xyz_euler_zyz);
	}

	return( Pt_CONTINUE );
}


int
start_wnd_irp6_on_track_xyz_angle_axis( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_xyz_angle_axis_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_xyz_angle_axis, widget, cbinfo);
		ui_state.is_wind_irp6ot_xyz_angle_axis_open=true;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_xyz_angle_axis);
	}

	return( Pt_CONTINUE );
}


int
clear_wnd_irp6_on_track_inc_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_inc_open=false;
	return( Pt_CONTINUE );

	}


int
clear_wnd_irp6_on_track_int_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_int_open=false;
	return( Pt_CONTINUE );

	}


int
clear_wnd_irp6ot_xyz_euler_zyz_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_xyz_euler_zyz_open=false;
	return( Pt_CONTINUE );

	}


int
clear_wnd_irp6ot_xyz_angle_axis_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_xyz_angle_axis_open=false;
	return( Pt_CONTINUE );

	}




int
clear_wnd_irp6ot_servo_algorithm_flag( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6ot_servo_algorithm_open=false;

	return( Pt_CONTINUE );

	}



int
import_wnd_irp6_on_track_inc( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char *tmp_ptgr, *tmp;
	double val;

	PtGetResource( ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0 );
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p0, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p1, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p2, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p3, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p4, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p5, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p6, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p7, Pt_ARG_NUMERIC_VALUE, &val , 0);

	return( Pt_CONTINUE );

	}


int
export_wnd_irp6_on_track_inc( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[ 200 ];

	double *wektor[8];

	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p5, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p6, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p7, Pt_ARG_NUMERIC_VALUE, &wektor[7], 0 );

      sprintf(buffer, "EDP_IRP6_OT INCREMENTAL POSITION\n %f %f %f %f %f %f %f %f",
		*wektor[0], *wektor[1], *wektor[2], *wektor[3],  *wektor[4], *wektor[5], *wektor[6], *wektor[7]);

	ui_msg.ui->message(buffer);

	return( Pt_CONTINUE );

	}


int
import_wnd_irp6_on_track_int( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource( ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0 );
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p1, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p2, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p3, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p4, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p5, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p6, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p7, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p8, Pt_ARG_NUMERIC_VALUE, &val , 0);

	return( Pt_CONTINUE );

	}





int
export_wnd_irp6_on_track_int( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[ 200 ];

	double *wektor[8];

	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p8, Pt_ARG_NUMERIC_VALUE, &wektor[7], 0 );

      sprintf(buffer, "EDP_IRP6_OT INTERNAL POSITION\n %f %f %f %f %f %f %f %f",
		*wektor[0], *wektor[1], *wektor[2], *wektor[3],  *wektor[4], *wektor[5], *wektor[6], *wektor[7]);

	ui_msg.ui->message(buffer);

	return( Pt_CONTINUE );

	}


int
import_wnd_irp6_on_track_xyz_euler_zyz( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource( ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0 );
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &val , 0);
    val = strtod( tmp, &tmp );
	PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p7, Pt_ARG_NUMERIC_VALUE, &val , 0);

	return( Pt_CONTINUE );

	}


int
export_wnd_irp6_on_track_xyz_euler_zyz( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[ 200 ];

	double *wektor[7];

	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );

      sprintf(buffer, "EDP_IRP6_OT XYZ_EULER_ZYZ POSITION\n %f %f %f %f %f %f %f",
		*wektor[0], *wektor[1], *wektor[2], *wektor[3],  *wektor[4], *wektor[5], *wektor[6]);

	ui_msg.ui->message(buffer);

	return( Pt_CONTINUE );

	}



int
start_wnd_irp6_on_track_xyz_angle_axis_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (	!ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open) // otworz okno
	{
		ApCreateModule (ABM_wnd_irp6_on_track_xyz_angle_axis_ts, widget, cbinfo);
		ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open=1;
	} else { // przelacz na okno
		PtWindowToFront (ABW_wnd_irp6_on_track_xyz_angle_axis_ts);
	}


	return( Pt_CONTINUE );

	}



// RUCHY RECZNE

// ---------------------------------------------------
// 1 os
// ---------------------------------------------------


// INCREMENTAL MOVES


int
init_wnd_irp6_on_track_inc( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{

			unblock_widget(ABW_PtPane_wind_irp6ot_inc_post_synchro_moves);
			if (!( ui_robot.irp6_on_track->read_motors(irp6ot_current_pos))) // Odczyt polozenia walow silnikow
				printf("Blad w read motors\n");

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p0, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[2] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[3] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p4, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[4] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p5, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[5] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p6, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[6] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p7, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[7] , 0);
			/*
			for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				irp6ot_desired_pos[i] = irp6ot_current_pos[i];
			*/
		}
		else
		{
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			block_widget(ABW_PtPane_wind_irp6ot_inc_post_synchro_moves);
		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

}


int
wnd_irp6ot_motors_copy_current_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[IRP6_ON_TRACK_NUM_OF_SERVOS], wektor[IRP6_ON_TRACK_NUM_OF_SERVOS];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{
			unblock_widget(ABW_PtPane_wind_irp6ot_inc_post_synchro_moves);

			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p0, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_motors_cur_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[7]) , 0);

			for (int i=0; i< IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p5, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p6, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p7, Pt_ARG_NUMERIC_VALUE, &wektor[7], 0 );

		}
		else
		{
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			block_widget(ABW_PtPane_wind_irp6ot_inc_post_synchro_moves);

		}
	}


	return( Pt_CONTINUE );

}



int
wnd_irp6ot_joints_copy_current_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[IRP6_ON_TRACK_NUM_OF_SERVOS], wektor[IRP6_ON_TRACK_NUM_OF_SERVOS];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{

			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p8, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[7]) , 0);

			for (int i=0; i< IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p8, Pt_ARG_NUMERIC_VALUE, &wektor[7], 0 );
		}
		else
		{

		}
	}

	return( Pt_CONTINUE );

	}




int
irp6ot_move_to_preset_position( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type==Ph_EV_KEY)
	{
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}


	// wychwytania ew. bledow ECP::robot
	try
	{

	if (ui_state.irp6_on_track.edp.pid!=-1) {

	 if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_on_track_preset_position_synchro) ||
		(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_synchro)) ||
		((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x73 ))
		)&&(ui_state.irp6_on_track.edp.is_synchronised)) {// powrot do pozycji synchronizacji
		 for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
		 {
		irp6ot_desired_pos[i] = 0.0;
          }
	} else  if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_on_track_preset_position_0)||
		(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_0))||
		((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x30 ))
		)&&(ui_state.irp6_on_track.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
		for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++) {
			 irp6ot_desired_pos[i] = ui_state.irp6_on_track.edp.preset_position[0][i];
			}
	} else  if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_on_track_preset_position_1)||
		(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_1))||
		((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x31 ))
		)&&(ui_state.irp6_on_track.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
		for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++) {
			 irp6ot_desired_pos[i] = ui_state.irp6_on_track.edp.preset_position[1][i];
		}
	} else  if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6_on_track_preset_position_2)||
		(ApName(ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_2))||
		((cbinfo->event->type==Ph_EV_KEY)&&(my_data->key_cap== 0x32 ))
		)&&(ui_state.irp6_on_track.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
		for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++) {
			 irp6ot_desired_pos[i] = ui_state.irp6_on_track.edp.preset_position[2][i];
		}
	}

	ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);

	} // end if (ui_state.irp6_on_track.edp.pid!=-1)
} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );
}


int
irp6ot_inc_motion( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	double *wektor[IRP6_ON_TRACK_NUM_OF_SERVOS];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{

if (ui_state.irp6_on_track.edp.pid!=-1) {

	 if (ui_state.irp6_on_track.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p0, Pt_ARG_NUMERIC_VALUE, &(wektor[0]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p1, Pt_ARG_NUMERIC_VALUE, &(wektor[1]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p2, Pt_ARG_NUMERIC_VALUE, &(wektor[2]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p3, Pt_ARG_NUMERIC_VALUE, &(wektor[3]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p4, Pt_ARG_NUMERIC_VALUE, &(wektor[4]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p5, Pt_ARG_NUMERIC_VALUE, &(wektor[5]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p6, Pt_ARG_NUMERIC_VALUE, &(wektor[6]), 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p7, Pt_ARG_NUMERIC_VALUE, &(wektor[7]), 0 );

		 for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			 {
		          irp6ot_desired_pos[i] = *wektor[i];
	          }
	} else {

		 for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
		 {
	          irp6ot_desired_pos[i] = 0.0;
         }
    }

	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_0l) {
		irp6ot_desired_pos[0]-=(*step1);
	} else

	if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_1l) {
		irp6ot_desired_pos[1]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_2l) {
		irp6ot_desired_pos[2]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_3l) {
		irp6ot_desired_pos[3]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_4l) {
		irp6ot_desired_pos[4]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_5l) {
		irp6ot_desired_pos[5]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_6l) {
		irp6ot_desired_pos[6]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_7l) {
		irp6ot_desired_pos[7]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_0r) {
		irp6ot_desired_pos[0]+=(*step1);
	} else

	if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_1r) {
		irp6ot_desired_pos[1]+=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_2r) {
		irp6ot_desired_pos[2]+=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_3r) {
		irp6ot_desired_pos[3]+=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_4r) {
		irp6ot_desired_pos[4]+=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_5r) {
		irp6ot_desired_pos[5]+=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_6r) {
		irp6ot_desired_pos[6]+=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_inc_7r) {
		irp6ot_desired_pos[7]+=(*step1);
	}


	ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);

	 if ((ui_state.irp6_on_track.edp.is_synchronised)&&(ui_state.is_wind_irp6ot_inc_open)) {     // by Y o dziwo niedziala poprawnie 	 if (ui_state.irp6_on_track.edp.is_synchronised)

		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p0, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[0] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[1] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[2] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[3] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p4, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[4] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p5, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[5] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p6, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[6] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_inc_p7, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[7] , 0);


	}
} // end if (ui_state.irp6_on_track.edp.pid!=-1)
} // end try

	CATCH_SECTION_UI

	return( Pt_CONTINUE );
}





int
init_wnd_irp6_on_track_int( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->read_joints(irp6ot_current_pos))) // Odczyt polozenia walow silnikow
				printf("Blad w read joints\n");

	// 	unblock_widget(ABW_PtPane_wind_irp6ot_int_post_synchro_moves);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[2] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p4, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[3] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p5, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[4] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p6, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[5] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p7, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[6] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_joints_cur_p8, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[7] , 0);

			for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				irp6ot_desired_pos[i] = irp6ot_current_pos[i];
		} else
		{
	// 		block_widget(ABW_PtPane_wind_irp6ot_int_post_synchro_moves);
		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );
}




int
irp6ot_int_motion( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	double *wektor[IRP6_ON_TRACK_NUM_OF_SERVOS];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if ((ui_state.irp6_on_track.edp.pid!=-1)&&(ui_state.irp6_on_track.edp.is_synchronised))
	{

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_p8, Pt_ARG_NUMERIC_VALUE, &wektor[7], 0 );

		 for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
		 {
	          irp6ot_desired_pos[i] = *wektor[i];
          }

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_1l) {
			irp6ot_desired_pos[0]-=(*step1);
		} else
		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_2l) {
			irp6ot_desired_pos[1]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_3l) {
			irp6ot_desired_pos[2]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_4l) {
			irp6ot_desired_pos[3]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_5l) {
			irp6ot_desired_pos[4]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_6l) {
			irp6ot_desired_pos[5]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_7l) {
			irp6ot_desired_pos[6]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_8l) {
			irp6ot_desired_pos[7]-=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_1r) {
			irp6ot_desired_pos[0]+=(*step1);
		} else
		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_2r) {
			irp6ot_desired_pos[1]+=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_3r) {
			irp6ot_desired_pos[2]+=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_4r) {
			irp6ot_desired_pos[3]+=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_5r) {
			irp6ot_desired_pos[4]+=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_6r) {
			irp6ot_desired_pos[5]+=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_7r) {
			irp6ot_desired_pos[6]+=(*step1);
		} else
		 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_int_8r) {
			irp6ot_desired_pos[7]+=(*step1);
		}

		ui_robot.irp6_on_track->move_joints(irp6ot_desired_pos);

		if (ui_state.is_wind_irp6ot_int_open)  // Czy robot jest zsynchronizowany?
		{

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[2] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p4, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[3] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p5, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[4] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p6, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[5] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p7, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[6] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_int_p8, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[7] , 0);
		}

	} // end if (ui_state.irp6_on_track.edp.pid!=-1)
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );
}





int
init_wnd_irp6_on_track_xyz_euler_zyz( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->read_xyz_euler_zyz(irp6ot_current_pos))) // Odczyt polozenia walow silnikow
				printf("Blad w read external\n");

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[2] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p4, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[3] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p5, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[4] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p6, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[5] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p7, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[6] , 0);

			for (int i = 0; i < 7; i++)
				irp6ot_desired_pos[i] = irp6ot_current_pos[i];
		} else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );
}



int
wnd_irp6ot_xyz_zyz_copy_cur_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[7], wektor[7];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_read_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]) , 0);

			for (int i=0; i< 7; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
		}
		else
		{

		}
	}

	return( Pt_CONTINUE );

	}





int
irp6ot_xyz_euler_zyz_motion( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	double *wektor[8];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	 if ( ui_state.irp6_on_track.edp.is_synchronised ) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );

	  for (int i = 0; i < 7; i++)
	       irp6ot_desired_pos[i] = *wektor[i];

	PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_1l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[0]-=(*step1);
	} else

	if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_2l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[1]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_3l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[2]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_4l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[3]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_5l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[4]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_6l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[5]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_7l) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[6]-=(*step1);
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_1r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[0]+=*step1;
	} else

	if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_2r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[1]+=*step1;
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_3r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[2]+=*step1;
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_4r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[3]+=*step1;
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_5r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[4]+=*step1;
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_6r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[5]+=*step1;
	} else

	 if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_euler_zyz_7r) {
	// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
		irp6ot_desired_pos[6]+=*step1;
	}
		ui_robot.irp6_on_track->move_xyz_euler_zyz(irp6ot_desired_pos);

		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[0] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[1] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[2] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[3] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[4] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[5] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_p7, Pt_ARG_NUMERIC_VALUE, &irp6ot_desired_pos[6] , 0);
   }
   } // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}





int
init_wnd_irp6_on_track_xyz_angle_axis( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double alfa, kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->read_xyz_angle_axis(irp6ot_current_pos))) // Odczyt polozenia walow silnikow
				printf("Blad w read_xyz_angle_axis\n");

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p1, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p2, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p3, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[2] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p8, Pt_ARG_NUMERIC_VALUE, &irp6ot_current_pos[6] , 0);

			alfa = sqrt(irp6ot_current_pos[3]*irp6ot_current_pos[3]
				+irp6ot_current_pos[4]*irp6ot_current_pos[4]
				+irp6ot_current_pos[5]*irp6ot_current_pos[5]);

			kx = irp6ot_current_pos[3]/alfa;
			ky = irp6ot_current_pos[4]/alfa;
			kz = irp6ot_current_pos[5]/alfa;

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p4, Pt_ARG_NUMERIC_VALUE, &kx , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p5, Pt_ARG_NUMERIC_VALUE, &ky , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p6, Pt_ARG_NUMERIC_VALUE, &kz , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p7, Pt_ARG_NUMERIC_VALUE, &alfa , 0);

			for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				irp6ot_desired_pos[i] = irp6ot_current_pos[i];
		}
		else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

return( Pt_CONTINUE );
}



int
wnd_irp6ot_xyz_aa_copy_current_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[8], wektor[8];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_read_p8, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[7]) , 0);

			for (int i=0; i< 8; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p8, Pt_ARG_NUMERIC_VALUE, &wektor[7], 0 );

		}
		else
		{

		}
	}

	return( Pt_CONTINUE );

	}




int
irp6ot_xyz_angle_axis_motion( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

{

	double *wektor_ptgr[8], wektor[8];
	double *krok;
	double wl; double l_eps = 0;
	double kx, ky, kz;
	// double alfa;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if ( ui_state.irp6_on_track.edp.is_synchronised )
	{

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p1, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p2, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p3, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p4, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p5, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p6, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p7, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[6], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p8, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[7], 0 );

		for (int i=0; i< 8; i++)
		{
			wektor[i] = *wektor_ptgr[i];
		}

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_step, Pt_ARG_NUMERIC_VALUE, &krok, 0 );

		// wektor przesuniecia
		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_1l)
			wektor[0]-=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_1r)
			wektor[0]+=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_2l)
			wektor[1]-=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_2r)
			wektor[1]+=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_3l)
			wektor[2]-=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_3r)
			wektor[2]+=(*krok);

		// parametry wersora obrotu


		// kat obrotu i chwytak
		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_7l)
			wektor[6]-=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_8l)
			wektor[7]-=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_7r)
			wektor[6]+=(*krok);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6ot_xyz_angle_axis_8r)
			wektor[7]+=(*krok);

		// sprawdzenie dlugosci wersora
		// w przypadku, gdy dlugosc wersora jest inna niz 1
		// parametry wersora zostaja przeskalowane

		kx = wektor[3];
		ky = wektor[4];
		kz = wektor[5];

		wl = sqrt(kx*kx + ky*ky + kz*kz);

		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			wektor[3] = kx/wl;
			wektor[4] = ky/wl;
			wektor[5] = kz/wl;

		}

		// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
		for(int i=0; i<6; i++)
		{
			irp6ot_desired_pos[i] = wektor[i];
			if( i >2)
				irp6ot_desired_pos[i] *= wektor[6];
		}
		irp6ot_desired_pos[6] = wektor[7];

		// zlecenie wykonania ruchu
		ui_robot.irp6_on_track->move_xyz_angle_axis(irp6ot_desired_pos);

		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6] , 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_p8, Pt_ARG_NUMERIC_VALUE, &wektor[7] , 0);

	}
	else
	{
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );
}


int
EDP_irp6_on_track_synchronise( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try
	{
		// dla robota irp6_on_track

		if ((ui_state.irp6_on_track.edp.state > 0)&&
			 (ui_state.irp6_on_track.edp.is_synchronised == false))
		{
			ui_robot.irp6_on_track->ecp->synchronise();
			ui_state.irp6_on_track.edp.is_synchronised = ui_robot.irp6_on_track->ecp->is_synchronised();
		} else {
			// 	printf("EDP irp6_on_track niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	manage_interface();

	return( Pt_CONTINUE );

	}




int
init_wnd_irp6_on_track_xyz_angle_axis_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double tool_vector[6];
	double alfa, kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->read_tool_xyz_angle_axis(tool_vector))) // Odczyt polozenia walow silnikow
				printf("Blad w read external\n");

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p1, Pt_ARG_NUMERIC_VALUE, &tool_vector[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p2, Pt_ARG_NUMERIC_VALUE, &tool_vector[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p3, Pt_ARG_NUMERIC_VALUE, &tool_vector[2] , 0);

			alfa = sqrt(tool_vector[3]*tool_vector[3]
				+tool_vector[4]*tool_vector[4]
				+tool_vector[5]*tool_vector[5]);

			if (alfa==0){
				kx = -1;
				ky = 0;
				kz = 0;
			}
			else{
				kx = tool_vector[3]/alfa;
				ky = tool_vector[4]/alfa;
				kz = tool_vector[5]/alfa;
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p4, Pt_ARG_NUMERIC_VALUE, &kx , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p5, Pt_ARG_NUMERIC_VALUE, &ky , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p6, Pt_ARG_NUMERIC_VALUE, &kz , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p7, Pt_ARG_NUMERIC_VALUE, &alfa , 0);
		}
		else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}


int
wnd_irp6ot_xyz_aa_ts_copy_cur_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	 double *wektor_ptgr[7], wektor[7];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{

			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_read_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]) , 0);

			for (int i=0; i< 7; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0 );
		}
		else
		{

		}
	}

	return( Pt_CONTINUE );

	}




int
irp6ot_xyz_angle_axis_set_tool( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor_ptgr[7], wektor[7];
	double tool_vector[6];
	double wl; double l_eps = 0;
	double kx, ky, kz;


	// wychwytania ew. bledow ECP::robot
	try
	{
	if ( ui_state.irp6_on_track.edp.is_synchronised )
	{

		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p1, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p2, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p3, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p4, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p5, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p6, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_angle_axis_ts_p7, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[6], 0 );

		for (int i=0; i< 7; i++)
		{
			wektor[i] = *wektor_ptgr[i];
		}

		// sprawdzenie dlugosci wersora
		// w przypadku, gdy dlugosc wersora jest inna niz 1
		// parametry wersora zostaja przeskalowane

		kx = wektor[3];
		ky = wektor[4];
		kz = wektor[5];

		wl = sqrt(kx*kx + ky*ky + kz*kz);

		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			wektor[3] = kx/wl;
			wektor[4] = ky/wl;
			wektor[5] = kz/wl;
		}

		// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
		for(int i=0; i<6; i++)
		{
			tool_vector[i] = wektor[i];
			if( i >2)
				tool_vector[i] *= wektor[6];
		}

		// zlecenie wykonania ruchu
		ui_robot.irp6_on_track->set_tool_xyz_angle_axis(tool_vector);

	}
	else
	{
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}




int
init_wnd_irp6_on_track_xyz_euler_zyz_ts( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double tool_vector[6];

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->read_tool_xyz_euler_zyz(tool_vector))) // Odczyt polozenia walow silnikow
				printf("Blad w read external\n");

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p1, Pt_ARG_NUMERIC_VALUE, &tool_vector[0] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p2, Pt_ARG_NUMERIC_VALUE, &tool_vector[1] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p3, Pt_ARG_NUMERIC_VALUE, &tool_vector[2] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p4, Pt_ARG_NUMERIC_VALUE, &tool_vector[3] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p5, Pt_ARG_NUMERIC_VALUE, &tool_vector[4] , 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p6, Pt_ARG_NUMERIC_VALUE, &tool_vector[5] , 0);

		} else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}



int
wnd_irp6ot_xyz_zyz_ts_copy_cur_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[6], wektor[6];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);

			for (int i=0; i< 6; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
			PtSetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );
		}
		else
		{

		}
	}

	return( Pt_CONTINUE );

	}



int
irp6ot_xyz_euler_zyz_set_tool( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor[6];
	double tool_vector[6];


	// wychwytania ew. bledow ECP::robot
	try
	{
	if ( ui_state.irp6_on_track.edp.is_synchronised )
	{
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0 );
		PtGetResource(ABW_PtNumericFloat_wind_irp6ot_xyz_euler_zyz_ts_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0 );

		for(int i=0; i<6; i++)
		{
			tool_vector[i] = *wektor[i];
		}

		// zlecenie wykonania ruchu
		ui_robot.irp6_on_track->set_tool_xyz_euler_zyz(tool_vector);

	}
	else
	{
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}





int
init_wnd_irp6_on_track_kinematic( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	BYTE model_no;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->get_kinematic(&model_no))) // Odczyt polozenia walow silnikow
				printf("Blad w read external\n");

			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_read_kinematic_model_no, Pt_ARG_NUMERIC_VALUE, model_no , 0);
		} else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}





int
irp6ot_kinematic_set( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	BYTE *model_no_tmp;
	BYTE model_no_output;

	// wychwytania ew. bledow ECP::robot
	try
	{
	if ( ui_state.irp6_on_track.edp.is_synchronised )
	{

		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_kinematic_model_no, Pt_ARG_NUMERIC_VALUE, &model_no_tmp, 0 );

		model_no_output = *model_no_tmp;

		// zlecenie wykonania ruchu
		ui_robot.irp6_on_track->set_kinematic(model_no_output);

	}
	else
	{
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}

int
init_wnd_irp6_on_track_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	BYTE servo_alg_no[IRP6_ON_TRACK_NUM_OF_SERVOS];
	BYTE servo_par_no[IRP6_ON_TRACK_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try
	{
	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
		{
			if (!(ui_robot.irp6_on_track->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
				printf("Blad w on_track get_servo_algorithm\n");

			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_2, Pt_ARG_NUMERIC_VALUE, servo_alg_no[1] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_3, Pt_ARG_NUMERIC_VALUE, servo_alg_no[2] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_4, Pt_ARG_NUMERIC_VALUE, servo_alg_no[3] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_5, Pt_ARG_NUMERIC_VALUE, servo_alg_no[4] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_6, Pt_ARG_NUMERIC_VALUE, servo_alg_no[5] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_7, Pt_ARG_NUMERIC_VALUE, servo_alg_no[6] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_8, Pt_ARG_NUMERIC_VALUE, servo_alg_no[7] , 0);

			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_2, Pt_ARG_NUMERIC_VALUE, servo_par_no[1] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_3, Pt_ARG_NUMERIC_VALUE, servo_par_no[2] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_4, Pt_ARG_NUMERIC_VALUE, servo_par_no[3] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_5, Pt_ARG_NUMERIC_VALUE, servo_par_no[4] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_6, Pt_ARG_NUMERIC_VALUE, servo_par_no[5] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_7, Pt_ARG_NUMERIC_VALUE, servo_par_no[6] , 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_8, Pt_ARG_NUMERIC_VALUE, servo_par_no[7] , 0);

		} else
		{

		}
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}




int
wnd_irp6ot_seralg_copy_current_to_desired( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	BYTE *wektor_ptgr[IRP6_ON_TRACK_NUM_OF_SERVOS], *wektor2_ptgr[IRP6_ON_TRACK_NUM_OF_SERVOS],
		wektor[IRP6_ON_TRACK_NUM_OF_SERVOS], wektor2[IRP6_ON_TRACK_NUM_OF_SERVOS];

	if (ui_state.irp6_on_track.edp.pid!=-1)
	{
		if (ui_state.irp6_on_track.edp.is_synchronised ) // Czy robot jest zsynchronizowany?
		{


			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_alg_8, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[7]) , 0);

			for (int i=0; i< IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, wektor[0], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_2, Pt_ARG_NUMERIC_VALUE, wektor[1], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_3, Pt_ARG_NUMERIC_VALUE, wektor[2], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_4, Pt_ARG_NUMERIC_VALUE, wektor[3], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_5, Pt_ARG_NUMERIC_VALUE, wektor[4], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_6, Pt_ARG_NUMERIC_VALUE, wektor[5], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_7, Pt_ARG_NUMERIC_VALUE, wektor[6], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_8, Pt_ARG_NUMERIC_VALUE, wektor[7], 0 );

			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[0]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_2, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[1]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_3, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[2]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_4, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[3]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_5, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[4]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_6, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[5]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_7, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[6]) , 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_read_par_8, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[7]) , 0);

			for (int i=0; i< IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				wektor2[i] = *wektor2_ptgr[i];
			}

			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, wektor2[0], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_2, Pt_ARG_NUMERIC_VALUE, wektor2[1], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_3, Pt_ARG_NUMERIC_VALUE, wektor2[2], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_4, Pt_ARG_NUMERIC_VALUE, wektor2[3], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_5, Pt_ARG_NUMERIC_VALUE, wektor2[4], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_6, Pt_ARG_NUMERIC_VALUE, wektor2[5], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_7, Pt_ARG_NUMERIC_VALUE, wektor2[6], 0 );
			PtSetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_8, Pt_ARG_NUMERIC_VALUE, wektor2[7], 0 );

		}
		else
		{
			// Wygaszanie elementow przy niezsynchronizowanym robocie


		}
	}


	return( Pt_CONTINUE );

	}


int
irp6ot_servo_algorithm_set( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	BYTE *servo_alg_no_tmp [IRP6_ON_TRACK_NUM_OF_SERVOS];
	BYTE servo_alg_no_output[IRP6_ON_TRACK_NUM_OF_SERVOS];
	BYTE *servo_par_no_tmp [IRP6_ON_TRACK_NUM_OF_SERVOS];
	BYTE servo_par_no_output[IRP6_ON_TRACK_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try
	{
	if ( ui_state.irp6_on_track.edp.is_synchronised )
	{

		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_2, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[1], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_3, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[2], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_4, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[3], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_5, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[4], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_6, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[5], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_7, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[6], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_alg_8, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[7], 0 );

		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_2, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[1], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_3, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[2], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_4, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[3], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_5, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[4], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_6, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[5], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_7, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[6], 0 );
		PtGetResource(ABW_PtNumericInteger_wnd_irp6ot_servo_algorithm_par_8, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[7], 0 );

		for(int i=0; i<IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
		{
			servo_alg_no_output[i] = *servo_alg_no_tmp[i];
			servo_par_no_output[i] = *servo_par_no_tmp[i];
		}

		// zlecenie wykonania ruchu
		ui_robot.irp6_on_track->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

	}
	else
	{
	}
	} // end try
	CATCH_SECTION_UI

	return( Pt_CONTINUE );

	}



int
EDP_irp6_on_track_create( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	short tmp;
	char tmp_string[100];
	char tmp2_string[100];

	FILE* file;					// do sprawdzenia czy istnieje /net/node_name/dev/TWOJ_ROBOT
	controller_state_t robot_controller_initial_state_tmp;

	try { // dla bledow robot :: ECP_error

	// dla robota irp6_on_track
	if (ui_state.irp6_on_track.edp.state == 0)
	{
		sprintf(tmp_string,  "/dev/name/global/%s", ui_state.irp6_on_track.edp.hardware_busy_attach_point);

		sprintf(tmp2_string, "/dev/name/global/%s", ui_state.irp6_on_track.edp.network_resourceman_attach_point);

		// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
		if((!(ui_state.irp6_on_track.edp.test_mode)) && ( access(tmp_string, R_OK)== 0  )
			|| (access(tmp2_string, R_OK)== 0 )
		)
		{
			ui_msg.ui->message("edp_irp6_on_track already exists");
		} else {
			ui_state.irp6_on_track.edp.node_nr = config->return_node_number(ui_state.irp6_on_track.edp.node_name);

			ui_robot.irp6_on_track = new ui_common_robot(
					*config, ui_msg.all_ecp,
					ROBOT_IRP6_ON_TRACK);

			ui_state.irp6_on_track.edp.pid = ui_robot.irp6_on_track->ecp->get_EDP_pid();

			if (ui_state.irp6_on_track.edp.pid<0)
			{
				fprintf( stderr, "EDP spawn failed: %s\n", strerror( errno ));
				delete ui_robot.irp6_on_track;
			} else {  // jesli spawn sie powiodl

				tmp = 0;
			 	// kilka sekund  (~1) na otworzenie urzadzenia
				while((ui_state.irp6_on_track.edp.reader_fd = name_open(ui_state.irp6_on_track.edp.network_reader_attach_point,
					NAME_FLAG_ATTACH_GLOBAL))  < 0)
					if((tmp++)<CONNECT_RETRY) {
						delay(CONNECT_DELAY);
					} else {
					   perror("blad odwolania do READER_OT");
					   break;
					}
				// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
				ui_robot.irp6_on_track->get_controller_state(&robot_controller_initial_state_tmp);

				ui_state.irp6_on_track.edp.state = 1; // edp wlaczone reader czeka na start

				ui_state.irp6_on_track.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
			}
		}
	}


	} // end try
	CATCH_SECTION_UI

	manage_interface();

	return( Pt_CONTINUE );

	}



int
EDP_irp6_on_track_slay( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// dla robota irp6_on_track
	if (ui_state.irp6_on_track.edp.state>0)
	 { // jesli istnieje EDP
		if (name_close(ui_state.irp6_on_track.edp.reader_fd) == -1) {
			fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n",
					__FILE__, __LINE__, strerror(errno));
		}
		delete ui_robot.irp6_on_track;

		if (SignalKill(ui_state.irp6_on_track.edp.node_nr, ui_state.irp6_on_track.edp.pid, 0, SIGTERM, 0, 0) == -1) {
			perror("UI(EDP_on_track) SignalKill()");
		}
		ui_state.irp6_on_track.edp.state = 0; // edp wylaczone
		ui_state.irp6_on_track.edp.is_synchronised = false;

		ui_state.irp6_on_track.edp.pid = -1;
		ui_state.irp6_on_track.edp.reader_fd = -1;

		close_all_irp6ot_windows (NULL, NULL, NULL);
	}

	// modyfikacja menu
	manage_interface();

	return( Pt_CONTINUE );

	}


int
close_all_irp6ot_windows( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	close_wnd_irp6_on_track_inc(NULL, NULL, NULL);
	close_wnd_irp6_on_track_int(NULL, NULL, NULL);
	close_wnd_irp6_on_track_xyz_angle_axis(NULL, NULL, NULL);
	close_wnd_irp6_on_track_xyz_angle_axis_ts(NULL, NULL, NULL);
	close_wnd_irp6_on_track_xyz_euler_zyz(NULL, NULL, NULL);
	close_wnd_irp6_on_track_xyz_euler_zyz_ts(NULL, NULL, NULL);
	close_wnd_irp6_on_track_kinematic(NULL, NULL, NULL);
	close_wnd_irp6_on_track_servo_algorithm (NULL, NULL, NULL);

	return( Pt_CONTINUE );

	}




int
pulse_reader_irp6ot_start( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_irp6ot_start_exec_pulse()) {
		process_control_window_init(widget, apinfo, cbinfo);
	}

	return( Pt_CONTINUE );

	}



bool pulse_reader_irp6ot_start_exec_pulse ()
{

	if (ui_state.irp6_on_track.edp.state == 1)
	{
		pulse_reader_execute( ui_state.irp6_on_track.edp.reader_fd, READER_START, 0);
		ui_state.irp6_on_track.edp.state = 2;
		return true;
	}

	return false;
}



int
pulse_reader_irp6ot_stop( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_irp6ot_stop_exec_pulse()) {
		process_control_window_init(widget, apinfo, cbinfo);
	}

	return( Pt_CONTINUE );

	}


bool pulse_reader_irp6ot_stop_exec_pulse ()
{

	if (ui_state.irp6_on_track.edp.state == 2)
	{
		pulse_reader_execute( ui_state.irp6_on_track.edp.reader_fd, READER_STOP, 0);
		ui_state.irp6_on_track.edp.state = 1;
		return true;
	}

	return false;
}



int
pulse_reader_irp6ot_trigger( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_irp6ot_trigger_exec_pulse()) process_control_window_init(widget, apinfo, cbinfo);

	return( Pt_CONTINUE );

	}


bool pulse_reader_irp6ot_trigger_exec_pulse ()
{

	if (ui_state.irp6_on_track.edp.state == 2)
	{
		pulse_reader_execute( ui_state.irp6_on_track.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}

	return false;
}

int
pulse_ecp_irp6_on_track( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.irp6_on_track.edp.is_synchronised)
	{ // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui_state.irp6_on_track.ecp.trigger_fd<0)
		{

			short tmp = 0;
	 		// kilka sekund  (~1) na otworzenie urzadzenia
			// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem

			ualarm( (useconds_t)( SIGALRM_TIMEOUT), 0);
			while( (ui_state.irp6_on_track.ecp.trigger_fd = name_open(ui_state.irp6_on_track.ecp.network_trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
			{
				if (errno == EINTR) break;
				if ((tmp++)<CONNECT_RETRY) {
					delay(CONNECT_DELAY);
				} else {
					perror("blad odwolania do ECP_TRIGGER\n");
				};
			}
			// odwolanie alarmu
			ualarm( (useconds_t)( 0), 0);
		}

		if (ui_state.irp6_on_track.ecp.trigger_fd>=0) {
			if (MsgSendPulse (ui_state.irp6_on_track.ecp.trigger_fd , sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {

				fprintf( stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",  strerror( errno ) );
				delay(1000);
			}
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return( Pt_CONTINUE );

	}


// aktualizacja ustawien przyciskow
int
process_control_window_irp6ot_section_init (bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (ui_state.irp6_on_track.edp.state<=0) {// edp wylaczone
		block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
	} else {
		if (ui_state.irp6_on_track.edp.state==1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start=true;
			unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
			block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
			block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		} else if (ui_state.irp6_on_track.edp.state==2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop=true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger=true;
			block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
			unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
			unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		}
	}

	ui_state.irp6_on_track.edp.last_state=ui_state.irp6_on_track.edp.state;

	return 1;

}


int
reload_irp6ot_configuration()
{
	char* tmp, *tmp1;
	char tmp_string[50];
	char tmp2_string[3];

	// jesli IRP6 on_track ma byc aktywne
	if ((ui_state.irp6_on_track.is_active = config->return_int_value("is_irp6_on_track_active")) == 1)
	{
		// ini_con->create_ecp_irp6_on_track (ini_con->ui->ecp_irp6_on_track_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active)
		{
			delete [] ui_state.irp6_on_track.ecp.network_trigger_attach_point;
			ui_state.irp6_on_track.ecp.network_trigger_attach_point =config->return_attach_point_name
				(configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.irp6_on_track.ecp.section_name);

			ui_state.irp6_on_track.ecp.pid = -1;
	 		ui_state.irp6_on_track.ecp.trigger_fd = -1;
	 	}

		switch (ui_state.irp6_on_track.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6_on_track (ini_con->ui->edp_irp6_on_track_section);

				ui_state.irp6_on_track.edp.pid = -1;
				ui_state.irp6_on_track.edp.reader_fd = -1;
				ui_state.irp6_on_track.edp.state = 0;

				for (int i=0; i<3; i++)
				{
					itoa( i, tmp2_string, 10 );

					strcpy(tmp_string,"preset_position_");
					strcat(tmp_string, tmp2_string);
					if (config->exists(tmp_string, ui_state.irp6_on_track.edp.section_name))
					{
						tmp1 = tmp = config->return_string_value(tmp_string, ui_state.irp6_on_track.edp.section_name);
						 for (int j=0; j<8; j++)
						{
							ui_state.irp6_on_track.edp.preset_position[i][j] = strtod( tmp1, &tmp1 );
						}
						delete[] tmp;
					} else {
						 for (int j=0; j<8; j++)
						{
							ui_state.irp6_on_track.edp.preset_position[i][j] = 0.0;
						}
					}
				}

				if (config->exists("test_mode", ui_state.irp6_on_track.edp.section_name))
					ui_state.irp6_on_track.edp.test_mode = config->return_int_value("test_mode", ui_state.irp6_on_track.edp.section_name);
				else
					ui_state.irp6_on_track.edp.test_mode = 0;

				delete [] ui_state.irp6_on_track.edp.hardware_busy_attach_point;
				ui_state.irp6_on_track.edp.hardware_busy_attach_point = config->return_string_value
					("hardware_busy_attach_point", ui_state.irp6_on_track.edp.section_name);

				delete [] ui_state.irp6_on_track.edp.network_resourceman_attach_point;
				ui_state.irp6_on_track.edp.network_resourceman_attach_point = config->return_attach_point_name
					(configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.irp6_on_track.edp.section_name);

				delete [] ui_state.irp6_on_track.edp.network_reader_attach_point;
				ui_state.irp6_on_track.edp.network_reader_attach_point = config->return_attach_point_name
					(configurator::CONFIG_SERVER, "reader_attach_point", ui_state.irp6_on_track.edp.section_name);

				delete [] ui_state.irp6_on_track.edp.node_name;
				ui_state.irp6_on_track.edp.node_name = config->return_string_value ("node_name", ui_state.irp6_on_track.edp.section_name);
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}

	} else // jesli  irp6 on_track ma byc nieaktywne
	{
			switch (ui_state.irp6_on_track.edp.state)
		{
			case -1:
			case 0:
				ui_state.irp6_on_track.edp.state = -1;
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	} // end irp6_on_track

	return 1;
}


int
manage_interface_irp6ot ()
{

	switch (ui_state.irp6_on_track.edp.state)
	{

		case -1:
			ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track, NULL);
		break;
		case 0:
			ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_unload,
				ABN_mm_irp6_on_track_pre_synchro_moves, ABN_mm_irp6_on_track_post_synchro_moves,
				 ABN_mm_irp6_on_track_tool_specification,  ABN_mm_irp6_on_track_preset_positions,
				 ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
			ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track, ABN_mm_irp6_on_track_edp_load, NULL);

		break;
		case 1:
		case 2:
			ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track, NULL);
			//ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			// jesli robot jest zsynchronizowany
			if (	ui_state.irp6_on_track.edp.is_synchronised)
			{
				ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_pre_synchro_moves, NULL);
				ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);

				switch (ui_state.mp.state)
				{
					case UI_MP_NOT_PERMITED_TO_RUN:
					case UI_MP_PERMITED_TO_RUN:
						ApModifyItemState( &robot_menu, AB_ITEM_NORMAL,  ABN_mm_irp6_on_track_edp_unload,
							ABN_mm_irp6_on_track_post_synchro_moves, ABN_mm_irp6_on_track_tool_specification,
							ABN_mm_irp6_on_track_preset_positions, ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
						ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_load, NULL);
					break;
					case UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState( &robot_menu, AB_ITEM_NORMAL,
							ABN_mm_irp6_on_track_post_synchro_moves, ABN_mm_irp6_on_track_tool_specification,
							ABN_mm_irp6_on_track_preset_positions, ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
						ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_load, ABN_mm_irp6_on_track_edp_unload, NULL);
					break;
					case UI_MP_TASK_RUNNING:
					case UI_MP_TASK_PAUSED:
						ApModifyItemState( &robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
							ABN_mm_irp6_on_track_post_synchro_moves, ABN_mm_irp6_on_track_preset_positions,
							ABN_mm_irp6_on_track_tool_specification,  ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
					break;
					default:
					break;
				}

			} else		// jesli robot jest niezsynchronizowany
			{
				ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track_edp_unload,
					ABN_mm_irp6_on_track_pre_synchro_moves, NULL);
				ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_load, NULL);
				ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
			}
		break;
		default:
		break;

	}

	return 1;
}
