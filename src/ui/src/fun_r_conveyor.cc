/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <errno.h>
#include <process.h>
#include <math.h>

#include <boost/bind.hpp>

#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui_class.h"
// #include "ui/ui.h"
// Konfigurator.
// #include "lib/configurator.h"
#include "ui/ui_ecp_r_tfg_and_conv.h"

#include "lib/robot_consts/conveyor_const.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern Ui ui;

// zamykanie okien ruchow recznych dla robota conveyor

int close_wind_conveyor_moves(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.conveyor.is_wind_conveyor_moves_open) {
		PtDestroyWidget(ABW_wnd_conveyor_moves);
	}

	return (Pt_CONTINUE);

}

int start_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui.conveyor.is_wind_conv_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_conveyor_servo_algorithm, widget, cbinfo);
		ui.conveyor.is_wind_conv_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_conveyor_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int close_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.conveyor.is_wind_conv_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_conveyor_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int start_wind_conveyor_moves(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui.conveyor.is_wind_conveyor_moves_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_conveyor_moves, widget, cbinfo);
		ui.conveyor.is_wind_conveyor_moves_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_conveyor_moves);
	}

	return (Pt_CONTINUE);

}

int clear_wind_conveyor_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.conveyor.is_wind_conveyor_moves_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_conveyor_servo_algorithm_flag(PtWidget_t *widget,
		ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.conveyor.is_wind_conv_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}

// dla robota conveyor

int wind_conveyor_moves_init(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((ui.conveyor.state.edp.pid != -1)
				&& (ui.conveyor.is_wind_conveyor_moves_open)) {
			if (ui.conveyor.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos);
				unblock_widget(ABW_PtButton_wind_conveyor_moves_inc_exec);

				unblock_widget(ABW_PtButton_wind_conveyor_moves_int_left);
				unblock_widget(ABW_PtButton_wind_conveyor_moves_int_right);
				unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_step);
				unblock_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_pos);
				unblock_widget(ABW_PtButton_wind_conveyor_moves_int_exec);

				ui.conveyor.ui_ecp_robot->read_motors(
						ui.conveyor.conveyor_current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(
						ABW_PtNumericFloat_wind_conveyor_moves_read_motor_pos,
						Pt_ARG_NUMERIC_VALUE,
						&ui.conveyor.conveyor_current_pos[0], 0);

				ui.conveyor.ui_ecp_robot->read_joints(
						ui.conveyor.conveyor_current_pos);

				PtSetResource(
						ABW_PtNumericFloat_wind_conveyor_moves_read_int_pos,
						Pt_ARG_NUMERIC_VALUE,
						&ui.conveyor.conveyor_current_pos[0], 0);

			} else {
				block_widget(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos);
				block_widget(ABW_PtButton_wind_conveyor_moves_inc_exec);

				block_widget(ABW_PtButton_wind_conveyor_moves_int_left);
				block_widget(ABW_PtButton_wind_conveyor_moves_int_right);
				block_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_step);
				block_widget(ABW_PtNumericFloat_wind_conveyor_moves_int_pos);
				block_widget(ABW_PtButton_wind_conveyor_moves_int_exec);
			}
			PtDamageWidget(ABW_wnd_conveyor_moves);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int conveyor_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	// wychwytania ew. bledow ECP::robot
	try {

		if (ui.conveyor.state.edp.pid != -1) {

			if ((((ApName(ApWidget(cbinfo))
					== ABN_mm_conveyor_preset_position_synchro) || (ApName(
					ApWidget(cbinfo))
					== ABN_mm_all_robots_preset_position_synchro))
					|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
							== 0x73)))
					&& (ui.conveyor.state.edp.is_synchronised)) {
				// powrot do pozycji synchronizacji
				for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
					ui.conveyor.conveyor_desired_pos[i] = 0.0;
				}

			} else if ((((ApName(ApWidget(cbinfo))
					== ABN_mm_conveyor_preset_position_0) || (ApName(ApWidget(
					cbinfo)) == ABN_mm_all_robots_preset_position_0))
					|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
							== 0x30)))
					&& (ui.conveyor.state.edp.is_synchronised)) {
				// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
				for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
					ui.conveyor.conveyor_desired_pos[i]
							= ui.conveyor.state.edp.preset_position[0][i];
				}
			} else if ((((ApName(ApWidget(cbinfo))
					== ABN_mm_conveyor_preset_position_1) || (ApName(ApWidget(
					cbinfo)) == ABN_mm_all_robots_preset_position_1))
					|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
							== 0x31)))
					&& (ui.conveyor.state.edp.is_synchronised)) {
				// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
				for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
					ui.conveyor.conveyor_desired_pos[i]
							= ui.conveyor.state.edp.preset_position[1][i];
				}
			} else if ((((ApName(ApWidget(cbinfo))
					== ABN_mm_conveyor_preset_position_2) || (ApName(ApWidget(
					cbinfo)) == ABN_mm_all_robots_preset_position_2))
					|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
							== 0x32)))
					&& (ui.conveyor.state.edp.is_synchronised)) {
				// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
				for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
					ui.conveyor.conveyor_desired_pos[i]
							= ui.conveyor.state.edp.preset_position[2][i];
				}
			}

			ui.conveyor.ui_ecp_robot->move_motors(
					ui.conveyor.conveyor_desired_pos);

		}

	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int wind_conveyor_moves_move(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	double *wektor_ptgr, conveyor_desired_pos_motors[6],
			conveyor_desired_pos_int[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {

		if (ui.conveyor.state.edp.pid != -1) {

			// incremental
			if ((widget == ABW_PtButton_wind_conveyor_moves_inc_left)
					|| (widget == ABW_PtButton_wind_conveyor_moves_inc_right)
					|| (widget == ABW_PtButton_wind_conveyor_moves_inc_exec)) {

				if (ui.conveyor.state.edp.is_synchronised) {
					PtGetResource(
							ABW_PtNumericFloat_wind_conveyor_moves_inc_pos,
							Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					conveyor_desired_pos_motors[0] = (*wektor_ptgr);
				} else {
					conveyor_desired_pos_motors[0] = 0.0;
				}

				PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_step,
						Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_conveyor_moves_inc_left) {
					conveyor_desired_pos_motors[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_conveyor_moves_inc_right) {
					conveyor_desired_pos_motors[0] += (*step1);
				}

				ui.conveyor.ui_ecp_robot->move_motors(
						conveyor_desired_pos_motors);

			}

			// internal
			if ((widget == ABW_PtButton_wind_conveyor_moves_int_left)
					|| (widget == ABW_PtButton_wind_conveyor_moves_int_right)
					|| (widget == ABW_PtButton_wind_conveyor_moves_int_exec)) {
				if (ui.conveyor.state.edp.is_synchronised) {
					PtGetResource(
							ABW_PtNumericFloat_wind_conveyor_moves_int_pos,
							Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					conveyor_desired_pos_int[0] = (*wektor_ptgr);
				}

				PtGetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_step,
						Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_conveyor_moves_int_left) {
					conveyor_desired_pos_int[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_conveyor_moves_int_right) {
					conveyor_desired_pos_int[0] += (*step1);
				}
				ui.conveyor.ui_ecp_robot->move_joints(conveyor_desired_pos_int);
			}

			// odswierzenie pozycji robota
			if ((ui.conveyor.state.edp.is_synchronised)
					&& (ui.conveyor.is_wind_conveyor_moves_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_inc_pos,
						Pt_ARG_NUMERIC_VALUE, &conveyor_desired_pos_motors[0],
						0);
				PtSetResource(ABW_PtNumericFloat_wind_conveyor_moves_int_pos,
						Pt_ARG_NUMERIC_VALUE, &conveyor_desired_pos_int[0], 0);

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int EDP_conveyor_synchronise(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	EDP_irp6_postumentcreate_int(widget, apinfo, cbinfo);

	ui.conveyor.eb.command(boost::bind(EDP_conveyor_synchronise_int, widget,
			apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_conveyor_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_on_track

		if ((ui.conveyor.state.edp.state > 0)
				&& (ui.conveyor.state.edp.is_synchronised == false)) {
			ui.conveyor.ui_ecp_robot->ecp->synchronise();
			ui.conveyor.state.edp.is_synchronised
					= ui.conveyor.ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("EDP conveyor niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	ui.manage_interface();

	return (Pt_CONTINUE);

}

int init_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[CONVEYOR_NUM_OF_SERVOS];
	uint8_t servo_par_no[CONVEYOR_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (ui.conveyor.state.edp.pid != -1) {
			if (ui.conveyor.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui.conveyor.ui_ecp_robot->get_servo_algorithm(servo_alg_no,
						servo_par_no);

				PtSetResource(
						ABW_PtNumericInteger_wnd_conv_servo_algorithm_read_alg_1,
						Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);

				PtSetResource(
						ABW_PtNumericInteger_wnd_conv_servo_algorithm_read_par_1,
						Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int conv_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[CONVEYOR_NUM_OF_SERVOS];
	uint8_t servo_alg_no_output[CONVEYOR_NUM_OF_SERVOS];
	uint8_t *servo_par_no_tmp[CONVEYOR_NUM_OF_SERVOS];
	uint8_t servo_par_no_output[CONVEYOR_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (ui.conveyor.state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_alg_1,
					Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_conv_servo_algorithm_par_1,
					Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);

			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			ui.conveyor.ui_ecp_robot->set_servo_algorithm(servo_alg_no_output,
					servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int EDP_conveyor_create(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	EDP_irp6_postumentcreate_int(widget, apinfo, cbinfo);

	ui.conveyor.eb.command(boost::bind(EDP_conveyor_create_int, widget, apinfo,
			cbinfo));

	return (Pt_CONTINUE);

}

int EDP_conveyor_create_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota conveyor
		if (ui.conveyor.state.edp.state == 0) {
			ui.conveyor.state.edp.state = 0;
			ui.conveyor.state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui.conveyor.state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string
					+= ui.conveyor.state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(ui.conveyor.state.edp.test_mode)) && (access(
					tmp_string.c_str(), R_OK) == 0)) || (access(
					tmp2_string.c_str(), R_OK) == 0)) {
				ui.ui_msg->message(lib::NON_FATAL_ERROR,
						"edp_conveyor already exists");
			} else if (ui.check_node_existence(ui.conveyor.state.edp.node_name,
					std::string("edp_conveyor"))) {
				ui.conveyor.state.edp.node_nr = ui.config->return_node_number(
						ui.conveyor.state.edp.node_name.c_str());
				{
					boost::unique_lock<boost::mutex> lock(
							ui.process_creation_mtx);
					ui.conveyor.ui_ecp_robot = new ui_tfg_and_conv_robot(
							*ui.config, *ui.all_ecp_msg, lib::ROBOT_CONVEYOR);

				}
				ui.conveyor.state.edp.pid
						= ui.conveyor.ui_ecp_robot->ecp->get_EDP_pid();

				if (ui.conveyor.state.edp.pid < 0) {
					ui.conveyor.state.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui.conveyor.ui_ecp_robot;
				} else { // jesli spawn sie powiodl
					ui.conveyor.state.edp.state = 1;
					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia
					while ((ui.conveyor.state.edp.reader_fd
							= name_open(
									ui.conveyor.state.edp.network_reader_attach_point.c_str(),
									NAME_FLAG_ATTACH_GLOBAL)) < 0)
						if ((tmp++) < CONNECT_RETRY)
							delay(CONNECT_DELAY);
						else {
							perror("blad odwolania do READER_C");
							break;
						}

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;
					ui.conveyor.ui_ecp_robot->get_controller_state(
							robot_controller_initial_state_tmp);

					//ui.conveyor.state.edp.state = 1; // edp wlaczone reader czeka na start
					ui.conveyor.state.edp.is_synchronised
							= robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try
	CATCH_SECTION_UI

	ui.manage_interface();

	return (Pt_CONTINUE);

}

int EDP_conveyor_slay(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//	EDP_irp6_postumentcreate_int(widget, apinfo, cbinfo);

	ui.conveyor.eb.command(boost::bind(EDP_conveyor_slay_int, widget, apinfo,
			cbinfo));

	return (Pt_CONTINUE);

}

int EDP_conveyor_slay_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{
	int pt_res;
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// dla robota conveyor
	if (ui.conveyor.state.edp.state > 0) { // jesli istnieje EDP
		if (ui.conveyor.state.edp.reader_fd >= 0) {
			if (name_close(ui.conveyor.state.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_conv, %s:%d, name_close(): %s\n",
						__FILE__, __LINE__, strerror(errno));
			}
		}

		delete ui.conveyor.ui_ecp_robot;
		ui.conveyor.state.edp.state = 0; // edp wylaczone
		ui.conveyor.state.edp.is_synchronised = false;

		ui.conveyor.state.edp.pid = -1;
		ui.conveyor.state.edp.reader_fd = -1;
		pt_res = PtEnter(0);
		close_wind_conveyor_moves(NULL, NULL, NULL);
		close_wnd_conveyor_servo_algorithm(NULL, NULL, NULL);
		if (pt_res >= 0)
			PtLeave(0);
	}

	// modyfikacja menu
	ui.manage_interface();

	return (Pt_CONTINUE);

}

int pulse_reader_conv_start(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.conveyor.pulse_reader_start_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_conv_stop(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.conveyor.pulse_reader_stop_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_conv_trigger(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.conveyor.pulse_reader_trigger_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_ecp_conveyor(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.conveyor.state.edp.is_synchronised > 0) { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui.conveyor.state.ecp.trigger_fd < 0) {

			short tmp = 0;
			// kilka sekund  (~1) na otworzenie urzadzenia
			// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem
			ualarm((useconds_t) (SIGALRM_TIMEOUT), 0);
			while ((ui.conveyor.state.ecp.trigger_fd = name_open(
					ui.conveyor.state.ecp.network_trigger_attach_point.c_str(),
					NAME_FLAG_ATTACH_GLOBAL)) < 0) {
				if (errno == EINTR)
					break;
				if ((tmp++) < CONNECT_RETRY)
					delay(CONNECT_DELAY);
				else {
					perror("blad odwolania do ECP_TRIGGER");
				}
			}
			// odwolanie alarmu
			ualarm((useconds_t) (0), 0);
		}

		if (ui.conveyor.state.ecp.trigger_fd >= 0) {
			if (MsgSendPulse(ui.conveyor.state.ecp.trigger_fd,
					sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value)
					== -1) {

				fprintf(stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",
						strerror(errno));
				delay(1000);
			}
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return (Pt_CONTINUE);

}

