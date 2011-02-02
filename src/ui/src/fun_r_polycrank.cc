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
#include "robot/polycrank/const_polycrank.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"


/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

// zamykanie okien ruchow recznych dla robota irp6_postument
int EDP_polycrank_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("EDP_polycrank_create\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.polycrank->edp_create();

	return (Pt_CONTINUE);
}

int EDP_polycrank_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("EDP_polycrank_slay\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.polycrank->EDP_slay_int();

	return (Pt_CONTINUE);
}


int EDP_polycrank_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("EDP_polycrank_synchronise\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.polycrank->synchronise();

	return (Pt_CONTINUE);
}


int start_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("start_wnd_polycrank_int\n");

	// eliminate 'unreferenced' warnings
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.polycrank->is_wind_polycrank_int_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_polycrank_int, widget, cbinfo);
		interface.polycrank->is_wind_polycrank_int_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_polycrank_int);
	}

	return (Pt_CONTINUE);
}

int wnd_polycrank_joints_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("wnd_polycrank_joints_copy_current_to_desired\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	//double *wektor_ptgr[NUM_OF_SERVOS], wektor[NUM_OF_SERVOS];
	double *wektor_ptgr[lib::polycrank::NUM_OF_SERVOS], wektor[lib::polycrank::NUM_OF_SERVOS];

	if (interface.polycrank->state.edp.pid != -1)
	{
		if (interface.polycrank->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < lib::polycrank::NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

		} else
		{
		}
	}

	return (Pt_CONTINUE);
}

int polycrank_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("polycrank_int_motion\n");

	double *wektor[lib::polycrank::NUM_OF_SERVOS];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if ((interface.polycrank->state.edp.pid != -1) && (interface.polycrank->state.edp.is_synchronised))
	{

		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

		for (int i = 0; i < lib::polycrank::NUM_OF_SERVOS; i++) {
			interface.polycrank->desired_pos[i] = *wektor[i];
		}

		PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_polycrank_int_1l)
		{
			interface.polycrank->desired_pos[0] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_2l)
		{
			interface.polycrank->desired_pos[1] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_3l)
		{
			interface.polycrank->desired_pos[2] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_4l)
		{
			interface.polycrank->desired_pos[3] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_5l)
		{
			interface.polycrank->desired_pos[4] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_6l)
		{
			interface.polycrank->desired_pos[5] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_7l)
		{
			interface.polycrank->desired_pos[6] -= (*step1);
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_1r)
		{
			interface.polycrank->desired_pos[0] += *step1;
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_2r)
		{
			interface.polycrank->desired_pos[1] += *step1;
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_3r)
		{
			interface.polycrank->desired_pos[2] += *step1;
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_4r)
		{
			interface.polycrank->desired_pos[3] += *step1;
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_5r)
		{
			interface.polycrank->desired_pos[4] += *step1;
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_6r)
		{
			interface.polycrank->desired_pos[5] += *step1;
		}
		else if (ApName(ApWidget(cbinfo))== ABN_PtButton_wind_polycrank_int_7r)
		{
			interface.polycrank->desired_pos[6] += *step1;
		}

		double desired_pos_int[7];

		for (int i = 0; i < lib::polycrank::NUM_OF_SERVOS; i++) {
			desired_pos_int[i] =  0.5;
		}

		interface.polycrank->ui_ecp_robot->move_joints(interface.polycrank->desired_pos);

		if (interface.polycrank->is_wind_polycrank_int_open) // Czy robot jest zsynchronizowany?
		{
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p1, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p2, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p3, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p4, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p5, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p6, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p7, Pt_ARG_NUMERIC_VALUE, &interface.polycrank->desired_pos[6], 0);
		}

	} // end if (interface.irp6m_m->state.edp.pid!=-1)
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);
}

int init_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("init_wnd_polycrank_int\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.polycrank->state.edp.pid != -1) {
		if (interface.polycrank->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			//if (!(interface.polycrank->ui_ecp_robot->read_joints(interface.polycrank->current_pos))) // Odczyt polozenia walow silnikow
			//printf("Blad w read joints\n");

			interface.polycrank->ui_ecp_robot->read_joints(interface.polycrank->current_pos); // Odczyt polozenia walow silnikow

			// 	interface.unblock_widget(ABW_PtPane_wind_irp6m_int_post_synchro_moves);

			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p1,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p2,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p3,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p4,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p5,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p6,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_polycrank_joints_cur_p7,Pt_ARG_NUMERIC_VALUE, &interface.polycrank->current_pos[6], 0);

			for (int i = 0; i < lib::polycrank::NUM_OF_SERVOS; i++)
			interface.polycrank->desired_pos[i] = interface.polycrank->current_pos[i];
		} else {
			// 	interface.block_widget(ABW_PtPane_wind_irp6m_int_post_synchro_moves);
		}
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);
}

int import_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("import_wnd_polycrank_int\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p1, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p2, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p3, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p4, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p5, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p6, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_polycrank_int_p7, Pt_ARG_NUMERIC_VALUE, &val, 0);

	delete[] tmp;

	return (Pt_CONTINUE);
}

int export_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("export_wnd_polycrank_int\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[7];

	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
	PtGetResource(ABW_PtNumericFloat_wind_polycrank_int_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

	sprintf(buffer, "edp_POLYCRANK INTERNAL POSITION\n %f %f %f %f %f %f %f",
			*wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5], *wektor[6] );

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);
}

int clear_wnd_polycrank_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("clear_wnd_polycrank_int_flag\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.polycrank->is_wind_polycrank_int_open = false;
	//interface.irp6m_m->is_wind_irp6m_int_open = false;

	return (Pt_CONTINUE);
}


int close_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	//printf("close_wnd_polycrank_int\n");

	// eliminate 'unreferenced' warnings
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;


	if (interface.polycrank->is_wind_polycrank_int_open) {
		PtDestroyWidget(ABW_wnd_polycrank_int);
	}

	return (Pt_CONTINUE);
}

