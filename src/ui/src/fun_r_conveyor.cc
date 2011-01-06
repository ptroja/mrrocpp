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
// #include "ui/src/ui.h"
// Konfigurator.
// #include "base/lib/configurator.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"

#include "robot/conveyor/const_conveyor.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

// zamykanie okien ruchow recznych dla robota conveyor

int close_wind_conveyor_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.conveyor->is_wind_conveyor_moves_open) {
		PtDestroyWidget(ABW_wnd_conveyor_moves);
	}

	return (Pt_CONTINUE);

}

int start_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.conveyor->is_wind_conv_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_conveyor_servo_algorithm, widget, cbinfo);
		interface.conveyor->is_wind_conv_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_conveyor_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int close_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.conveyor->is_wind_conv_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_conveyor_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int start_wind_conveyor_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.conveyor->is_wind_conveyor_moves_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_conveyor_moves, widget, cbinfo);
		interface.conveyor->is_wind_conveyor_moves_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_conveyor_moves);
	}

	return (Pt_CONTINUE);

}

int clear_wind_conveyor_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.conveyor->is_wind_conveyor_moves_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_conveyor_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.conveyor->is_wind_conv_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}

// dla robota conveyor

int wind_conveyor_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((interface.conveyor->state.edp.pid != -1) && (interface.conveyor->is_wind_conveyor_moves_open)) {
			if (interface.conveyor->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos);
				interface.unblock_widget(ABW_PtButton_wind_conveyor_moves_inc_exec);

				interface.unblock_widget(ABW_PtButton_wind_conveyor_moves_int_left);
				interface.unblock_widget(ABW_PtButton_wind_conveyor_moves_int_right);
				interface.unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_step);
				interface.unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_pos);
				interface.unblock_widget(ABW_PtButton_wind_conveyor_moves_int_exec);

				interface.conveyor->ui_ecp_robot->read_motors(interface.conveyor->current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_read_motor_pos, Pt_ARG_NUMERIC_VALUE, &interface.conveyor->current_pos[0], 0);

				interface.conveyor->ui_ecp_robot->read_joints(interface.conveyor->current_pos);

				PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_read_int_pos, Pt_ARG_NUMERIC_VALUE, &interface.conveyor->current_pos[0], 0);

			} else {
				interface.block_widget(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos);
				interface.block_widget(ABW_PtButton_wind_conveyor_moves_inc_exec);

				interface.block_widget(ABW_PtButton_wind_conveyor_moves_int_left);
				interface.block_widget(ABW_PtButton_wind_conveyor_moves_int_right);
				interface.block_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_step);
				interface.block_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_pos);
				interface.block_widget(ABW_PtButton_wind_conveyor_moves_int_exec);
			}
			PtDamageWidget(ABW_wnd_conveyor_moves);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}


int wind_conveyor_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	double *wektor_ptgr, desired_pos_motors[6], desired_pos_int[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {

		if (interface.conveyor->state.edp.pid != -1) {

			// incremental
			if ((widget == ABW_PtButton_wind_conveyor_moves_inc_left) || (widget
					== ABW_PtButton_wind_conveyor_moves_inc_right) || (widget
					== ABW_PtButton_wind_conveyor_moves_inc_exec)) {

				if (interface.conveyor->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					desired_pos_motors[0] = (*wektor_ptgr);
				} else {
					desired_pos_motors[0] = 0.0;
				}

				PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_conveyor_moves_inc_left) {
					desired_pos_motors[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_conveyor_moves_inc_right) {
					desired_pos_motors[0] += (*step1);
				}

				interface.conveyor->ui_ecp_robot->move_motors(desired_pos_motors);

			}

			// internal
			if ((widget == ABW_PtButton_wind_conveyor_moves_int_left) || (widget
					== ABW_PtButton_wind_conveyor_moves_int_right) || (widget
					== ABW_PtButton_wind_conveyor_moves_int_exec)) {
				if (interface.conveyor->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					desired_pos_int[0] = (*wektor_ptgr);
				}

				PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_conveyor_moves_int_left) {
					desired_pos_int[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_conveyor_moves_int_right) {
					desired_pos_int[0] += (*step1);
				}
				interface.conveyor->ui_ecp_robot->move_joints(desired_pos_int);
			}

			// odswierzenie pozycji robota
			if ((interface.conveyor->state.edp.is_synchronised) && (interface.conveyor->is_wind_conveyor_moves_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &desired_pos_motors[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &desired_pos_int[0], 0);

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int EDP_conveyor_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.conveyor->synchronise();

	return (Pt_CONTINUE);

}

int init_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[interface.conveyor->number_of_servos];
	uint8_t servo_par_no[interface.conveyor->number_of_servos];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.conveyor->state.edp.pid != -1) {
			if (interface.conveyor->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.conveyor->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no);

				PtSetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);

				PtSetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int conv_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[interface.conveyor->number_of_servos];
	uint8_t servo_alg_no_output[interface.conveyor->number_of_servos];
	uint8_t *servo_par_no_tmp[interface.conveyor->number_of_servos];
	uint8_t servo_par_no_output[interface.conveyor->number_of_servos];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.conveyor->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);

			for (int i = 0; i < interface.conveyor->number_of_servos; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			interface.conveyor->ui_ecp_robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int EDP_conveyor_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.conveyor->edp_create();

	return (Pt_CONTINUE);

}

int EDP_conveyor_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	EDP_irp6_postumentcreate_int(widget, apinfo, cbinfo);

	interface.conveyor->EDP_slay_int();
	return (Pt_CONTINUE);

}

int pulse_reader_conv_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.conveyor->pulse_reader_start_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_conv_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.conveyor->pulse_reader_stop_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_conv_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.conveyor->pulse_reader_trigger_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_ecp_conveyor(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.conveyor->pulse_ecp();
	return (Pt_CONTINUE);

}


int
conveyor_move_to_synchro_position( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


int
conveyor_move_to_preset_position_0( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


int
conveyor_move_to_preset_position_1( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


int
conveyor_move_to_preset_position_2( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}

