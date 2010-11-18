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

int close_wind_sarkofag_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.sarkofag->is_wind_sarkofag_moves_open) {
		PtDestroyWidget(ABW_wnd_sarkofag_moves);
	}

	return (Pt_CONTINUE);

}

int close_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.sarkofag->is_wind_sarkofag_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_sarkofag_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int EDP_sarkofag_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (interface.sarkofag->state.edp.state == 0) {
		interface.sarkofag->create_thread();
		interface.sarkofag->eb.command(boost::bind(EDP_sarkofag_create_int, widget, apinfo, cbinfo));
	}

	return (Pt_CONTINUE);

}

int EDP_sarkofag_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	//	sleep(10);
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota sarkofag
		if (interface.sarkofag->state.edp.state == 0) {

			interface.sarkofag->state.edp.state = 0;
			interface.sarkofag->state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += interface.sarkofag->state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += interface.sarkofag->state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(interface.sarkofag->state.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0))
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "edp_sarkofag already exists");
			} else if (interface.check_node_existence(interface.sarkofag->state.edp.node_name, "edp_sarkofag")) {

				interface.sarkofag->state.edp.node_nr = interface.config->return_node_number(interface.sarkofag->state.edp.node_name);

				{
					boost::unique_lock <boost::mutex> lock(interface.process_creation_mtx);

					interface.sarkofag->ui_ecp_robot
							= new ui::tfg_and_conv::EcpRobot(*interface.config, *interface.all_ecp_msg, lib::sarkofag::ROBOT_NAME);
				}

				interface.sarkofag->state.edp.pid = interface.sarkofag->ui_ecp_robot->ecp->get_EDP_pid();

				if (interface.sarkofag->state.edp.pid < 0) {

					interface.sarkofag->state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete interface.sarkofag->ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					interface.sarkofag->state.edp.state = 1;

					interface.sarkofag->connect_to_reader();

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					interface.sarkofag->ui_ecp_robot->get_controller_state(robot_controller_initial_state_tmp);

					//interface.sarkofag->state.edp.state = 1; // edp wlaczone reader czeka na start

					interface.sarkofag->state.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	interface.manage_interface();

	return 1;
}

int EDP_sarkofag_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->EDP_slay_int();

	return (Pt_CONTINUE);

}

int EDP_sarkofag_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->eb.command(boost::bind(EDP_sarkofag_synchronise_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_sarkofag_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota sarkofag_

		if ((interface.sarkofag->state.edp.state > 0) && (interface.sarkofag->state.edp.is_synchronised == false)) {
			interface.sarkofag->ui_ecp_robot->ecp->synchronise();
			interface.sarkofag->state.edp.is_synchronised = interface.sarkofag->ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp sarkofag niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	interface.manage_interface();

	return (Pt_CONTINUE);

}

int start_wind_sarkofag_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.sarkofag->is_wind_sarkofag_moves_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_sarkofag_moves, widget, cbinfo);
		interface.sarkofag->is_wind_sarkofag_moves_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_sarkofag_moves);
	}

	return (Pt_CONTINUE);

}

int sarkofag_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (interface.sarkofag->state.edp.pid != -1) {

		if ((((ApName(ApWidget(cbinfo)) == ABN_mm_sarkofag_preset_position_synchro) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_synchro)) || ((cbinfo->event->type == Ph_EV_KEY)
				&& (my_data->key_cap == 0x73))) && (interface.sarkofag->state.edp.is_synchronised)) {// powrot do pozycji synchronizacji
			for (int i = 0; i < lib::sarkofag::NUM_OF_SERVOS; i++) {
				interface.sarkofag->sarkofag_desired_pos[i] = 0.0;
			}
			interface.sarkofag->eb.command(boost::bind(sarkofag_execute_motor_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_sarkofag_preset_position_0) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_0)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x30))) && (interface.sarkofag->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::sarkofag::NUM_OF_SERVOS; i++) {
				interface.sarkofag->sarkofag_desired_pos[i] = interface.sarkofag->state.edp.preset_position[0][i];
			}
			interface.sarkofag->eb.command(boost::bind(sarkofag_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_sarkofag_preset_position_1) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_1)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x31))) && (interface.sarkofag->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::sarkofag::NUM_OF_SERVOS; i++) {
				interface.sarkofag->sarkofag_desired_pos[i] = interface.sarkofag->state.edp.preset_position[1][i];
			}
			interface.sarkofag->eb.command(boost::bind(sarkofag_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_sarkofag_preset_position_2) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_2)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x32))) && (interface.sarkofag->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < lib::sarkofag::NUM_OF_SERVOS; i++) {
				interface.sarkofag->sarkofag_desired_pos[i] = interface.sarkofag->state.edp.preset_position[2][i];
			}
			interface.sarkofag->eb.command(boost::bind(sarkofag_execute_joint_motion));
		}

		//	interface.sarkofag->ui_ecp_robot->move_motors(interface.sarkofag->sarkofag_desired_pos);

	} // end if (interface.sarkofag->state.edp.pid!=-1)


	return (Pt_CONTINUE);

}

int sarkofag_execute_motor_motion()
{
	try {

		interface.sarkofag->ui_ecp_robot->move_motors(interface.sarkofag->sarkofag_desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int sarkofag_execute_joint_motion()
{
	try {

		interface.sarkofag->ui_ecp_robot->move_joints(interface.sarkofag->sarkofag_desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int start_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.sarkofag->is_wind_sarkofag_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_sarkofag_servo_algorithm, widget, cbinfo);
		interface.sarkofag->is_wind_sarkofag_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_sarkofag_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int init_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[lib::sarkofag::NUM_OF_SERVOS];
	uint8_t servo_par_no[lib::sarkofag::NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.sarkofag->state.edp.pid != -1) {
			if (interface.sarkofag->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.sarkofag->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no);

				PtSetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);

				PtSetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int sarkofag_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[lib::sarkofag::NUM_OF_SERVOS];
	uint8_t servo_alg_no_output[lib::sarkofag::NUM_OF_SERVOS];
	uint8_t *servo_par_no_tmp[lib::sarkofag::NUM_OF_SERVOS];
	uint8_t servo_par_no_output[lib::sarkofag::NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.sarkofag->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);

			for (int i = 0; i < lib::sarkofag::NUM_OF_SERVOS; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			interface.sarkofag->ui_ecp_robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wind_sarkofag_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((interface.sarkofag->state.edp.pid != -1) && (interface.sarkofag->is_wind_sarkofag_moves_open)) {
			if (interface.sarkofag->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.unblock_widget(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_inc_exec);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_int_left);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_int_right);
				interface.unblock_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_step);
				interface.unblock_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_int_exec);

				interface.sarkofag->ui_ecp_robot->read_motors(interface.sarkofag->sarkofag_current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_read_motor_pos, Pt_ARG_NUMERIC_VALUE, &interface.sarkofag->sarkofag_current_pos[0], 0);

				interface.sarkofag->ui_ecp_robot->read_joints(interface.sarkofag->sarkofag_current_pos);

				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_read_int_pos, Pt_ARG_NUMERIC_VALUE, &interface.sarkofag->sarkofag_current_pos[0], 0);

			} else {
				interface.block_widget(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos);
				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_inc_exec);

				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_int_left);
				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_int_right);
				interface.block_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_step);
				interface.block_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos);
				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_int_exec);
			}
			PtDamageWidget(ABW_wnd_sarkofag_moves);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wind_sarkofag_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->is_wind_sarkofag_moves_open = false;

	return (Pt_CONTINUE);

}

int wind_sarkofag_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor_ptgr, sarkofag_desired_pos_motors[6], sarkofag_desired_pos_int[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {

		if (interface.sarkofag->state.edp.pid != -1) {

			// incremental
			if ((widget == ABW_PtButton_wind_sarkofag_moves_inc_left) || (widget
					== ABW_PtButton_wind_sarkofag_moves_inc_right) || (widget
					== ABW_PtButton_wind_sarkofag_moves_inc_exec)) {

				if (interface.sarkofag->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					sarkofag_desired_pos_motors[0] = (*wektor_ptgr);
				} else {
					sarkofag_desired_pos_motors[0] = 0.0;
				}

				PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_sarkofag_moves_inc_left) {
					sarkofag_desired_pos_motors[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_sarkofag_moves_inc_right) {
					sarkofag_desired_pos_motors[0] += (*step1);
				}

				interface.sarkofag->ui_ecp_robot->move_motors(sarkofag_desired_pos_motors);

			}

			// internal
			if ((widget == ABW_PtButton_wind_sarkofag_moves_int_left) || (widget
					== ABW_PtButton_wind_sarkofag_moves_int_right) || (widget
					== ABW_PtButton_wind_sarkofag_moves_int_exec)) {
				if (interface.sarkofag->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					sarkofag_desired_pos_int[0] = (*wektor_ptgr);
				}

				PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_sarkofag_moves_int_left) {
					sarkofag_desired_pos_int[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_sarkofag_moves_int_right) {
					sarkofag_desired_pos_int[0] += (*step1);
				}
				interface.sarkofag->ui_ecp_robot->move_joints(sarkofag_desired_pos_int);
			}

			// odswierzenie pozycji robota
			if ((interface.sarkofag->state.edp.is_synchronised) && (interface.sarkofag->is_wind_sarkofag_moves_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &sarkofag_desired_pos_motors[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &sarkofag_desired_pos_int[0], 0);

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wnd_sarkofag_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->is_wind_sarkofag_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}
