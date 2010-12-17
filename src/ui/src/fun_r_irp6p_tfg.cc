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
#include "base/lib/configurator.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

int close_wind_irp6p_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_tfg->is_wind_irp6p_tfg_moves_open) {
		PtDestroyWidget(ABW_wnd_irp6p_tfg_moves);
	}

	return (Pt_CONTINUE);

}

int close_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6p_tfg->is_wind_irp6p_tfg_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_irp6p_tfg_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_tfg->edp_create();

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_tfg->EDP_slay_int();

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_tfg->synchronise();

	return (Pt_CONTINUE);

}

int start_wind_irp6p_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_tfg->is_wind_irp6p_tfg_moves_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6p_tfg_moves, widget, cbinfo);
		interface.irp6p_tfg->is_wind_irp6p_tfg_moves_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6p_tfg_moves);
	}

	return (Pt_CONTINUE);

}

int irp6p_tfg_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (interface.irp6p_tfg->state.edp.pid != -1) {

		if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_synchro) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_synchro)) || ((cbinfo->event->type == Ph_EV_KEY)
				&& (my_data->key_cap == 0x73))) && (interface.irp6p_tfg->state.edp.is_synchronised)) {// powrot do pozycji synchronizacji
			for (int i = 0; i < lib::irp6p_tfg::NUM_OF_SERVOS; i++) {
				interface.irp6p_tfg->desired_pos[i] = 0.0;
			}
			interface.irp6p_tfg->eb.command(boost::bind(&ui::irp6p_tfg::UiRobot::execute_motor_motion, &(*interface.irp6p_tfg)));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_0) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_0)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x30))) && (interface.irp6p_tfg->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_tfg::NUM_OF_SERVOS; i++) {
				interface.irp6p_tfg->desired_pos[i] = interface.irp6p_tfg->state.edp.preset_position[0][i];
			}
			interface.irp6p_tfg->eb.command(boost::bind(&ui::irp6p_tfg::UiRobot::execute_joint_motion, &(*interface.irp6p_tfg)));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_1) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_1)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x31))) && (interface.irp6p_tfg->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_tfg::NUM_OF_SERVOS; i++) {
				interface.irp6p_tfg->desired_pos[i] = interface.irp6p_tfg->state.edp.preset_position[1][i];
			}
			interface.irp6p_tfg->eb.command(boost::bind(&ui::irp6p_tfg::UiRobot::execute_joint_motion, &(*interface.irp6p_tfg)));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_2) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_2)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x32))) && (interface.irp6p_tfg->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::irp6p_tfg::NUM_OF_SERVOS; i++) {
				interface.irp6p_tfg->desired_pos[i] = interface.irp6p_tfg->state.edp.preset_position[2][i];
			}
			interface.irp6p_tfg->eb.command(boost::bind(&ui::irp6p_tfg::UiRobot::execute_joint_motion, &(*interface.irp6p_tfg)));
		}

		//	interface.irp6p_tfg->ui_ecp_robot->move_motors(interface.irp6p_tfg->desired_pos);

	} // end if (interface.irp6p_tfg->state.edp.pid!=-1)


	return (Pt_CONTINUE);

}

int start_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6p_tfg->is_wind_irp6p_tfg_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6p_tfg_servo_algorithm, widget, cbinfo);
		interface.irp6p_tfg->is_wind_irp6p_tfg_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6p_tfg_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int init_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[lib::irp6p_tfg::NUM_OF_SERVOS];
	uint8_t servo_par_no[lib::irp6p_tfg::NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_tfg->state.edp.pid != -1) {
			if (interface.irp6p_tfg->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.irp6p_tfg->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no);

				PtSetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);

				PtSetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int irp6p_tfg_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[lib::irp6p_tfg::NUM_OF_SERVOS];
	uint8_t servo_alg_no_output[lib::irp6p_tfg::NUM_OF_SERVOS];
	uint8_t *servo_par_no_tmp[lib::irp6p_tfg::NUM_OF_SERVOS];
	uint8_t servo_par_no_output[lib::irp6p_tfg::NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.irp6p_tfg->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);

			for (int i = 0; i < lib::irp6p_tfg::NUM_OF_SERVOS; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			interface.irp6p_tfg->ui_ecp_robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wind_irp6p_tfg_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((interface.irp6p_tfg->state.edp.pid != -1) && (interface.irp6p_tfg->is_wind_irp6p_tfg_moves_open)) {
			if (interface.irp6p_tfg->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.unblock_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos);
				interface.unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_inc_exec);

				interface.unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_left);
				interface.unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_right);
				interface.unblock_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_step);
				interface.unblock_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos);
				interface.unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_exec);

				interface.irp6p_tfg->ui_ecp_robot->read_motors(interface.irp6p_tfg->current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_read_motor_pos, Pt_ARG_NUMERIC_VALUE, &interface.irp6p_tfg->current_pos[0], 0);

				interface.irp6p_tfg->ui_ecp_robot->read_joints(interface.irp6p_tfg->current_pos);

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_read_int_pos, Pt_ARG_NUMERIC_VALUE, &interface.irp6p_tfg->current_pos[0], 0);

			} else {
				interface.block_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos);
				interface.block_widget(ABW_PtButton_wind_irp6p_tfg_moves_inc_exec);

				interface.block_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_left);
				interface.block_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_right);
				interface.block_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_step);
				interface.block_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos);
				interface.block_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_exec);
			}
			PtDamageWidget(ABW_wnd_irp6p_tfg_moves);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wind_irp6p_tfg_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_tfg->is_wind_irp6p_tfg_moves_open = false;

	return (Pt_CONTINUE);

}

int wind_irp6p_tfg_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor_ptgr, desired_pos_motors[6], desired_pos_int[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {

		if (interface.irp6p_tfg->state.edp.pid != -1) {

			// incremental
			if ((widget == ABW_PtButton_wind_irp6p_tfg_moves_inc_left) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_inc_right) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_inc_exec)) {

				if (interface.irp6p_tfg->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					desired_pos_motors[0] = (*wektor_ptgr);
				} else {
					desired_pos_motors[0] = 0.0;
				}

				PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_irp6p_tfg_moves_inc_left) {
					desired_pos_motors[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_irp6p_tfg_moves_inc_right) {
					desired_pos_motors[0] += (*step1);
				}

				interface.irp6p_tfg->ui_ecp_robot->move_motors(desired_pos_motors);

			}

			// internal
			if ((widget == ABW_PtButton_wind_irp6p_tfg_moves_int_left) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_int_right) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_int_exec)) {
				if (interface.irp6p_tfg->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					desired_pos_int[0] = (*wektor_ptgr);
				}

				PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_irp6p_tfg_moves_int_left) {
					desired_pos_int[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_irp6p_tfg_moves_int_right) {
					desired_pos_int[0] += (*step1);
				}
				interface.irp6p_tfg->ui_ecp_robot->move_joints(desired_pos_int);
			}

			// odswierzenie pozycji robota
			if ((interface.irp6p_tfg->state.edp.is_synchronised) && (interface.irp6p_tfg->is_wind_irp6p_tfg_moves_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &desired_pos_motors[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &desired_pos_int[0], 0);

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_tfg_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6p_tfg->is_wind_irp6p_tfg_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}

