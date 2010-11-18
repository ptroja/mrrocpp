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

// #include "ui/src/ui.h"
// Konfigurator.
// #include "base/lib/configurator.h"
#include "robot/polycrank/const_polycrank.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

int init_wnd_polycrank_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	lib::Xyz_Euler_Zyz_vector tool_vector;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->read_tool_xyz_euler_zyz(
									tool_vector))) // Odczyt polozenia walow silnikow
			printf("Blad w read external\n");

			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p1,
					Pt_ARG_NUMERIC_VALUE, &(tool_vector.w[0]), 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p2,
					Pt_ARG_NUMERIC_VALUE, &(tool_vector.w[1]), 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p3,
					Pt_ARG_NUMERIC_VALUE, &(tool_vector.w[2]), 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p4,
					Pt_ARG_NUMERIC_VALUE, &(tool_vector.w[3]), 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p5,
					Pt_ARG_NUMERIC_VALUE, &(tool_vector.w[4]), 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p6,
					Pt_ARG_NUMERIC_VALUE, &(tool_vector.w[5]), 0);

		} else {

		}
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int clear_wnd_polycrank_xyz_euler_zyz_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_ts_open = 0;

	return (Pt_CONTINUE);

}

int wnd_polycrank_xyz_zyz_ts_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[6], wektor[6];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);

			for (int i = 0; i < 6; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int EDP_polycrank_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota irp6_mechatronika
		if (interface.irp6m_m->state.edp.state == 0) {

			interface.irp6m_m->state.edp.state = 0;
			interface.irp6m_m->state.edp.is_synchronised = false;

			std::string busy_attach_point("/dev/name/global/");
			busy_attach_point
			+= interface.irp6m_m->state.edp.hardware_busy_attach_point;

			std::string resourceman_attach_point("/dev/name/global/");
			resourceman_attach_point
			+= interface.irp6m_m->state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if ((!(interface.irp6m_m->state.edp.test_mode)) && (access(
									busy_attach_point.c_str(), R_OK) == 0) || (access(
									resourceman_attach_point.c_str(), R_OK) == 0)) {
				interface.ui_msg->message(lib::NON_FATAL_ERROR,
						"edp_irp6_mechatronika already exists");

			} else if (check_node_existence(interface.irp6m_m->state.edp.node_name, "edp_irp6_mechatronika")) {

				interface.irp6m_m->state.edp.node_nr = interface.config->return_node_number(
						interface.irp6m_m->state.edp.node_name);

				interface.irp6m_m->ui_ecp_robot = new EcpRobot(*interface.config,
						*interface.all_ecp_msg, lib::irp6m::ROBOT_NAME);

				interface.irp6m_m->state.edp.pid
				= interface.irp6m_m->ui_ecp_robot->ecp->get_EDP_pid();

				if (interface.irp6m_m->state.edp.pid < 0) {
					interface.irp6m_m->state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete interface.irp6m_m->ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					interface.irp6m_m->state.edp.state = 1;

					interface.polycrank->connect_to_reader();

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;
					interface.irp6m_m->ui_ecp_robot->get_controller_state(
							robot_controller_initial_state_tmp);

					//interface.irp6m_m->state.edp.state = 1; // edp wlaczone reader czeka na start
					interface.irp6m_m->state.edp.is_synchronised
					= robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try
	CATCH_SECTION_UI

	interface.manage_interface();

	return (Pt_CONTINUE);

}

int EDP_polycrank_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// dla robota irp6_mechatronika
	if (interface.irp6m_m->state.edp.state > 0) { // jesli istnieje EDP
		if (ui_state.polycrank.edp.reader_fd >= 0) {
			if (name_close(ui_state.polycrank.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_polycrank, %s:%d, name_close(): %s\n", __FILE__, __LINE__, strerror(errno));
			}
		}

		delete interface.irp6m_m->ui_ecp_robot;
		interface.irp6m_m->state.edp.state = 0; // edp wylaczone
		interface.irp6m_m->state.edp.is_synchronised = false;

		interface.irp6m_m->state.edp.pid = -1;
		interface.irp6m_m->state.edp.reader_fd = -1;

		close_all_irp6m_windows(NULL, NULL, NULL);
	}

	// modyfikacja menu
	interface.manage_interface();

	return (Pt_CONTINUE);

}

int close_all_polycrank_windows(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	close_wnd_irp6_mechatronika_inc(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_int(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_xyz_angle_axis(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_xyz_angle_axis_ts(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_xyz_euler_zyz(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_xyz_euler_zyz_ts(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_kinematic(NULL, NULL, NULL);
	close_wnd_irp6_mechatronika_servo_algorithm(NULL, NULL, NULL);

	return (Pt_CONTINUE);

}

int EDP_polycrank_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_mechatronika

		if ((interface.irp6m_m->state.edp.state > 0)
				&& (interface.irp6m_m->state.edp.is_synchronised == false)) {
			interface.irp6m_m->ui_ecp_robot->ecp->synchronise();
			interface.irp6m_m->state.edp.is_synchronised
			= interface.irp6m_m->ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_mechatronika niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	interface.manage_interface();

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_int_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_int, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_int_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_int);
	}

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_inc_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_inc, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_inc_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_inc);
	}

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_xyz_euler_zyz, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_xyz_euler_zyz);
	}

	return (Pt_CONTINUE);

}

int polycrank_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	// wychwytania ew. bledow ECP::robot
try {

	if (interface.irp6m_m->state.edp.pid != -1) {

		if ((((ApName(ApWidget(cbinfo))
										== ABN_mm_irp6_mechatronika_preset_position_synchro)
								|| (ApName(ApWidget(cbinfo))
										== ABN_mm_all_robots_preset_position_synchro))
						|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
										== 0x73)))
				&& (interface.irp6m_m->state.edp.is_synchronised)) {// powrot do pozycji synchronizacji
			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				interface.irp6m_m->irp6m_desired_pos[i] = 0;
			}
		} else if ((((ApName(ApWidget(cbinfo))
										== ABN_mm_irp6_mechatronika_preset_position_0) || (ApName(
												ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_0))
						|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
										== 0x30)))
				&& (interface.irp6m_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				interface.irp6m_m->irp6m_desired_pos[i]
				= interface.irp6m_m->state.edp.preset_position[0][i];
			}
		} else if ((((ApName(ApWidget(cbinfo))
										== ABN_mm_irp6_mechatronika_preset_position_1) || (ApName(
												ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_1))
						|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
										== 0x31)))
				&& (interface.irp6m_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				interface.irp6m_m->irp6m_desired_pos[i]
				= interface.irp6m_m->state.edp.preset_position[1][i];
			}
		} else if ((((ApName(ApWidget(cbinfo))
										== ABN_mm_irp6_mechatronika_preset_position_2) || (ApName(
												ApWidget(cbinfo)) == ABN_mm_all_robots_preset_position_2))
						|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
										== 0x32)))
				&& (interface.irp6m_m->state.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				interface.irp6m_m->irp6m_desired_pos[i]
				= interface.irp6m_m->state.edp.preset_position[2][i];
			}
		}

		interface.irp6m_m->ui_ecp_robot->move_motors(interface.irp6m_m->irp6m_desired_pos);

	} // end if (interface.irp6m_m->state.edp.pid!=-1)
} // end try

CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int start_wnd_polycrank_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_ts_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_xyz_euler_zyz_ts, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_ts_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_xyz_euler_zyz_ts);
	}

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_ts_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_xyz_angle_axis_ts, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_ts_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_xyz_angle_axis_ts);
	}

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_kinematic_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_kinematic, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_kinematic_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_kinematic);
	}

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6_mechatronika_servo_algorithm, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int pulse_reader_polycrank_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_irp6m_start_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_polycrank_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_irp6m_stop_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_polycrank_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (pulse_reader_irp6m_trigger_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_ecp_polycrank(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.polycrank->pulse_ecp();

	return (Pt_CONTINUE);

}

int wnd_polycrank_motors_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[NUM_OF_SERVOS], wektor[NUM_OF_SERVOS];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p0, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6m_inc_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor[5];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {

		if (interface.irp6m_m->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p0,
					Pt_ARG_NUMERIC_VALUE, &(wektor[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p1,
					Pt_ARG_NUMERIC_VALUE, &(wektor[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p2,
					Pt_ARG_NUMERIC_VALUE, &(wektor[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p3,
					Pt_ARG_NUMERIC_VALUE, &(wektor[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p4,
					Pt_ARG_NUMERIC_VALUE, &(wektor[4]), 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				interface.irp6m_m->irp6m_desired_pos[i] = *wektor[i];
			}
		} else {
			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				interface.irp6m_m->irp6m_desired_pos[i] = 0.0;
			}
		}

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_step,
				Pt_ARG_NUMERIC_VALUE, &step1, 0);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_0l) {
			interface.irp6m_m->irp6m_desired_pos[0] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_1l) {
			interface.irp6m_m->irp6m_desired_pos[1] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_2l) {
			interface.irp6m_m->irp6m_desired_pos[2] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_3l) {
			interface.irp6m_m->irp6m_desired_pos[3] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_4l) {
			interface.irp6m_m->irp6m_desired_pos[4] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_0r) {
			interface.irp6m_m->irp6m_desired_pos[0] += (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_1r) {
			interface.irp6m_m->irp6m_desired_pos[1] += (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_2r) {
			interface.irp6m_m->irp6m_desired_pos[2] += (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_3r) {
			interface.irp6m_m->irp6m_desired_pos[3] += (*step1);
		} else

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_inc_4r) {
			interface.irp6m_m->irp6m_desired_pos[4] += (*step1);
		}

		interface.irp6m_m->ui_ecp_robot->move_motors(interface.irp6m_m->irp6m_desired_pos);

		if ((interface.irp6m_m->state.edp.is_synchronised)
				&& (interface.irp6m_m->is_wind_irp6m_inc_open)) {

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p0,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p4,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[4], 0);

		}
	} // end if (interface.irp6m_m->state.edp.pid!=-1)
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int init_wnd_polycrank_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{

			interface.unblock_widget(ABW_PtPane_wind_irp6m_inc_post_synchro_moves);
			if (!(interface.irp6m_m->ui_ecp_robot->read_motors(interface.irp6m_m->irp6m_current_pos))) // Odczyt polozenia walow silnikow
			printf("Blad w read motors\n");

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p0,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_motors_cur_p4,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[4], 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++)
			interface.irp6m_m->irp6m_desired_pos[i] = interface.irp6m_m->irp6m_current_pos[i];
		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			interface.block_widget(ABW_PtPane_wind_irp6m_inc_post_synchro_moves);
		}
	}
} // end try

CATCH_SECTION_UI

return (Pt_CONTINUE);
}

int import_wnd_polycrank_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p0, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p1, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p2, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p3, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_inc_p4, Pt_ARG_NUMERIC_VALUE, &val, 0);

	return (Pt_CONTINUE);
}

int export_wnd_polycrank_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[5];

	PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);

	sprintf(buffer, "edp_IRP6_M INTERNAL POSITION\n %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);
}

int wnd_polycrank_joints_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[NUM_OF_SERVOS], wektor[NUM_OF_SERVOS];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);

		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6m_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor[NUM_OF_SERVOS];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if ((interface.irp6m_m->state.edp.pid != -1)
			&& (interface.irp6m_m->state.edp.is_synchronised)) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p1,
				Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p2,
				Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p3,
				Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p4,
				Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p5,
				Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);

		for (int i = 0; i < NUM_OF_SERVOS; i++) {
			interface.irp6m_m->irp6m_desired_pos[i] = *wektor[i];
		}

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_step,
				Pt_ARG_NUMERIC_VALUE, &step1, 0);

		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_irp6m_int_1l) {
			interface.irp6m_m->irp6m_desired_pos[0] -= (*step1);
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_2l) {
			interface.irp6m_m->irp6m_desired_pos[1] -= (*step1);
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_3l) {
			interface.irp6m_m->irp6m_desired_pos[2] -= (*step1);
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_4l) {
			interface.irp6m_m->irp6m_desired_pos[3] -= (*step1);
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_5l) {
			interface.irp6m_m->irp6m_desired_pos[4] -= (*step1);
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_1r) {
			interface.irp6m_m->irp6m_desired_pos[0] += *step1;
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_2r) {
			interface.irp6m_m->irp6m_desired_pos[1] += *step1;
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_3r) {
			interface.irp6m_m->irp6m_desired_pos[2] += *step1;
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_4r) {
			interface.irp6m_m->irp6m_desired_pos[3] += *step1;
		} else if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_int_5r) {
			interface.irp6m_m->irp6m_desired_pos[4] += *step1;
		}

		interface.irp6m_m->ui_ecp_robot->move_joints(interface.irp6m_m->irp6m_desired_pos);

		if (interface.irp6m_m->is_wind_irp6m_int_open) // Czy robot jest zsynchronizowany?
		{

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p4,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p5,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[4], 0);

		}

	} // end if (interface.irp6m_m->state.edp.pid!=-1)
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int init_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->read_joints(interface.irp6m_m->irp6m_current_pos))) // Odczyt polozenia walow silnikow
			printf("Blad w read joints\n");

			// 	interface.unblock_widget(ABW_PtPane_wind_irp6m_int_post_synchro_moves);

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p4,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_joints_cur_p5,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[4], 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++)
			interface.irp6m_m->irp6m_desired_pos[i] = interface.irp6m_m->irp6m_current_pos[i];
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
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p1, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p2, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p3, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p4, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_int_p5, Pt_ARG_NUMERIC_VALUE, &val, 0);

	return (Pt_CONTINUE);
}

int export_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[5];

	PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_int_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
	sprintf(buffer, "edp_IRP6_M INTERNAL POSITION\n %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);

}

int init_wnd_polycrank_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t model_no;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->get_kinematic(&model_no))) // Odczyt polozenia walow silnikow
			printf("Blad w read external\n");

			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_read_kinematic_model_no,
					Pt_ARG_NUMERIC_VALUE, model_no, 0);
		} else {

		}
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int irp6m_kinematic_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *model_no_tmp;
	uint8_t model_no_output;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_kinematic_model_no,
				Pt_ARG_NUMERIC_VALUE, &model_no_tmp, 0);

		model_no_output = *model_no_tmp;

		// zlecenie wykonania ruchu
		interface.irp6m_m->ui_ecp_robot->set_kinematic(model_no_output);

	} else {
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int wnd_polycrank_ser_alg_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	uint8_t *wektor_ptgr[NUM_OF_SERVOS], *wektor2_ptgr[NUM_OF_SERVOS], wektor[NUM_OF_SERVOS], wektor2[NUM_OF_SERVOS];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, wektor[0], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_2, Pt_ARG_NUMERIC_VALUE, wektor[1], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_3, Pt_ARG_NUMERIC_VALUE, wektor[2], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_4, Pt_ARG_NUMERIC_VALUE, wektor[3], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_5, Pt_ARG_NUMERIC_VALUE, wektor[4], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_2, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_3, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_4, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_5, Pt_ARG_NUMERIC_VALUE, &(wektor2_ptgr[4]), 0);

			for (int i = 0; i < NUM_OF_SERVOS; i++) {
				wektor2[i] = *wektor2_ptgr[i];
			}

			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, wektor2[0], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_2, Pt_ARG_NUMERIC_VALUE, wektor2[1], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_3, Pt_ARG_NUMERIC_VALUE, wektor2[2], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_4, Pt_ARG_NUMERIC_VALUE, wektor2[3], 0);
			PtSetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_5, Pt_ARG_NUMERIC_VALUE, wektor2[4], 0);

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie


		}
	}

	return (Pt_CONTINUE);

}

int init_wnd_polycrank_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[NUM_OF_SERVOS];
	uint8_t servo_par_no[NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->get_servo_algorithm(
									servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
			printf("Blad w mechatronika get_servo_algorithm\n");

			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_1,
					Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_2,
					Pt_ARG_NUMERIC_VALUE, servo_alg_no[1], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_3,
					Pt_ARG_NUMERIC_VALUE, servo_alg_no[2], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_4,
					Pt_ARG_NUMERIC_VALUE, servo_alg_no[3], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_alg_5,
					Pt_ARG_NUMERIC_VALUE, servo_alg_no[4], 0);

			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_1,
					Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_2,
					Pt_ARG_NUMERIC_VALUE, servo_par_no[1], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_3,
					Pt_ARG_NUMERIC_VALUE, servo_par_no[2], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_4,
					Pt_ARG_NUMERIC_VALUE, servo_par_no[3], 0);
			PtSetResource(
					ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_read_par_5,
					Pt_ARG_NUMERIC_VALUE, servo_par_no[4], 0);

		} else {

		}
	}
} // end try
CATCH_SECTION_UI

return 1;

}

int irp6m_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[NUM_OF_SERVOS];
	uint8_t servo_alg_no_output[NUM_OF_SERVOS];
	uint8_t *servo_par_no_tmp[NUM_OF_SERVOS];
	uint8_t servo_par_no_output[NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_1,
				Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_2,
				Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[1], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_3,
				Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[2], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_4,
				Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[3], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_alg_5,
				Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[4], 0);

		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_1,
				Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_2,
				Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[1], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_3,
				Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[2], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_4,
				Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[3], 0);
		PtGetResource(ABW_PtNumericInteger_wnd_irp6m_servo_algorithm_par_5,
				Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[4], 0);

		for (int i = 0; i < NUM_OF_SERVOS; i++) {
			servo_alg_no_output[i] = *servo_alg_no_tmp[i];
			servo_par_no_output[i] = *servo_par_no_tmp[i];
		}

		// zlecenie wykonania ruchu
		interface.irp6m_m->ui_ecp_robot->set_servo_algorithm(servo_alg_no_output,
				servo_par_no_output);

	} else {
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int wnd_polycrank_xyz_aa_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[7], wektor[7];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < 7; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6m_xyz_angle_axis_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor_ptgr[7], wektor[7];
	double *krok;
	double wl;
	double l_eps = 0;
	double kx, ky, kz;
	// double alfa;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p1,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p2,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p3,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p4,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p5,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p6,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p7,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[6], 0);

		for (int i = 0; i < 7; i++) {
			wektor[i] = *wektor_ptgr[i];
		}

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_step,
				Pt_ARG_NUMERIC_VALUE, &krok, 0);

		// wektor przesuniecia
		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_1l)
		wektor[0] -= (*krok);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_1r)
		wektor[0] += (*krok);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_2l)
		wektor[1] -= (*krok);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_2r)
		wektor[1] += (*krok);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_3l)
		wektor[2] -= (*krok);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_3r)
		wektor[2] += (*krok);

		// parametry wersora obrotu


		// kat obrotu
		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_7l)
		wektor[6] -= (*krok);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_angle_axis_7r)
		wektor[6] += (*krok);

		// sprawdzenie dlugosci wersora
		// w przypadku, gdy dlugosc wersora jest inna niz 1
		// parametry wersora zostaja przeskalowane

		kx = wektor[3];
		ky = wektor[4];
		kz = wektor[5];

		wl = sqrt(kx * kx + ky * ky + kz * kz);

		if ((wl > 1 + l_eps) || (wl < 1 - l_eps)) {
			wektor[3] = kx / wl;
			wektor[4] = ky / wl;
			wektor[5] = kz / wl;

		}

		// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
		for (int i = 0; i < 6; i++) {
			interface.irp6m_m->irp6m_desired_pos[i] = wektor[i];
			if (i > 2)
			interface.irp6m_m->irp6m_desired_pos[i] *= wektor[6];
		}

		// zlecenie wykonania ruchu
		interface.irp6m_m->ui_ecp_robot->move_xyz_angle_axis(interface.irp6m_m->irp6m_desired_pos);

		// odswiezenie zawartosci okna
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p1,
				Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p2,
				Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p3,
				Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p4,
				Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p5,
				Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p6,
				Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_p7,
				Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);

	} else {
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int init_wnd_polycrank_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
/* eliminate 'unreferenced' warnings */
{
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double alfa, kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->read_xyz_angle_axis(
									interface.irp6m_m->irp6m_current_pos))) // Odczyt polozenia walow silnikow
			printf("Blad w read read_xyz_angle_axis\n");

			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[0], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[1], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[2], 0);

			alfa = sqrt(interface.irp6m_m->irp6m_current_pos[3] * interface.irp6m_m->irp6m_current_pos[3]
					+ interface.irp6m_m->irp6m_current_pos[4] * interface.irp6m_m->irp6m_current_pos[4]
					+ interface.irp6m_m->irp6m_current_pos[5] * interface.irp6m_m->irp6m_current_pos[5]);

			kx = interface.irp6m_m->irp6m_current_pos[3] / alfa;
			ky = interface.irp6m_m->irp6m_current_pos[4] / alfa;
			kz = interface.irp6m_m->irp6m_current_pos[5] / alfa;

			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p4,
					Pt_ARG_NUMERIC_VALUE, &kx, 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p5,
					Pt_ARG_NUMERIC_VALUE, &ky, 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p6,
					Pt_ARG_NUMERIC_VALUE, &kz, 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_read_p7,
					Pt_ARG_NUMERIC_VALUE, &alfa, 0);

			for (int i = 0; i < 6; i++)
			interface.irp6m_m->irp6m_desired_pos[i] = interface.irp6m_m->irp6m_current_pos[i];
		} else {

		}
	}
} // end try
CATCH_SECTION_UI
return (Pt_CONTINUE);

}

int init_wnd_polycrank_xyz_aa_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double tool_vector[6];
	double alfa, kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->read_tool_xyz_angle_axis(
									tool_vector))) // Odczyt polozenia walow silnikow
			printf("Blad w read external\n");

			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p1,
					Pt_ARG_NUMERIC_VALUE, &tool_vector[0], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p2,
					Pt_ARG_NUMERIC_VALUE, &tool_vector[1], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p3,
					Pt_ARG_NUMERIC_VALUE, &tool_vector[2], 0);

			alfa = sqrt(tool_vector[3] * tool_vector[3] + tool_vector[4]
					* tool_vector[4] + tool_vector[5] * tool_vector[5]);

			if (alfa == 0) {
				kx = -1;
				ky = 0;
				kz = 0;
			} else {
				kx = tool_vector[3] / alfa;
				ky = tool_vector[4] / alfa;
				kz = tool_vector[5] / alfa;
			}

			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p4,
					Pt_ARG_NUMERIC_VALUE, &kx, 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p5,
					Pt_ARG_NUMERIC_VALUE, &ky, 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p6,
					Pt_ARG_NUMERIC_VALUE, &kz, 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p7,
					Pt_ARG_NUMERIC_VALUE, &alfa, 0);

		} else {

		}
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);
}

int irp6m_xyz_angle_axis_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor_ptgr[7], wektor[7];
	double tool_vector[6];
	double wl;
	double l_eps = 0;
	double kx, ky, kz;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p1,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p2,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p3,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p4,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p5,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p6,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p7,
				Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[6], 0);

		for (int i = 0; i < 7; i++) {
			wektor[i] = *wektor_ptgr[i];
		}

		// sprawdzenie dlugosci wersora
		// w przypadku, gdy dlugosc wersora jest inna niz 1
		// parametry wersora zostaja przeskalowane

		kx = wektor[3];
		ky = wektor[4];
		kz = wektor[5];

		wl = sqrt(kx * kx + ky * ky + kz * kz);

		if ((wl > 1 + l_eps) || (wl < 1 - l_eps)) {
			wektor[3] = kx / wl;
			wektor[4] = ky / wl;
			wektor[5] = kz / wl;

		}

		// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
		for (int i = 0; i < 6; i++) {
			tool_vector[i] = wektor[i];
			if (i > 2)
			tool_vector[i] *= wektor[6];
		}

		// zlecenie wykonania ruchu
		interface.irp6m_m->ui_ecp_robot->set_tool_xyz_angle_axis(tool_vector);

	} else {
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int wnd_polycrank_xyz_aa_ts_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[7], wektor[7];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_read_p7, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[6]), 0);

			for (int i = 0; i < 7; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_angle_axis_ts_p7, Pt_ARG_NUMERIC_VALUE, &wektor[6], 0);
		} else {

		}
	}

	return (Pt_CONTINUE);

}

int wnd_polycrank_xyz_zyz_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[6], wektor[6];

	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p1, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[0]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p2, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[1]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p3, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[2]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p4, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[3]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p5, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[4]), 0);
			PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p6, Pt_ARG_NUMERIC_VALUE, &(wektor_ptgr[5]), 0);

			for (int i = 0; i < 6; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

		} else {

		}
	}

	return (Pt_CONTINUE);

}

int irp6m_xyz_euler_zyz_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {

	if (interface.irp6m_m->state.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p1,
				Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p2,
				Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p3,
				Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p4,
				Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p5,
				Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p6,
				Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

		for (int i = 0; i < 6; i++)
		interface.irp6m_m->irp6m_desired_pos[i] = *wektor[i];

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_step,
				Pt_ARG_NUMERIC_VALUE, &step1, 0);

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_1l) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[0] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_2l) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[1] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_3l) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[2] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_4l) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[3] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_5l) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[4] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_6l) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[5] -= (*step1);
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_1r) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[0] += *step1;
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_2r) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[1] += *step1;
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_3r) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[2] += *step1;
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_4r) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[3] += *step1;
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_5r) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[4] += *step1;
		} else

		if (ApName(ApWidget(cbinfo))
				== ABN_PtButton_wind_irp6m_xyz_euler_zyz_6r) {
			// 	PtGetResource(ABW_PtNumericFloat_internal_step,Pt_ARG_NUMERIC_VALUE, &step1, 0 );
			interface.irp6m_m->irp6m_desired_pos[5] += *step1;
		}

		interface.irp6m_m->ui_ecp_robot->move_xyz_euler_zyz(interface.irp6m_m->irp6m_desired_pos);

		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p1,
				Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[0], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p2,
				Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[1], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p3,
				Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[2], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p4,
				Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[3], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p5,
				Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[4], 0);
		PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p6,
				Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_desired_pos[5], 0);

	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int init_wnd_polycrank_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.pid != -1) {
		if (interface.irp6m_m->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			if (!(interface.irp6m_m->ui_ecp_robot->read_xyz_euler_zyz(
									interface.irp6m_m->irp6m_current_pos))) // Odczyt polozenia walow silnikow
			printf("Blad w read external\n");

			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p1,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[0], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p2,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[1], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p3,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[2], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p4,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[3], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p5,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[4], 0);
			PtSetResource(
					ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_read_p6,
					Pt_ARG_NUMERIC_VALUE, &interface.irp6m_m->irp6m_current_pos[5], 0);

			for (int i = 0; i < 6; i++)
			interface.irp6m_m->irp6m_desired_pos[i] = interface.irp6m_m->irp6m_current_pos[i];
		} else {

		}
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int import_wnd_polycrank_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &val, 0);

	return (Pt_CONTINUE);

}

int export_wnd_polycrank_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char buffer[200];

	double *wektor[6];

	PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p1, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p2, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p3, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p4, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p5, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_p6, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

	sprintf(buffer, "edp_IRP6_M XYZ_EULER_ZYZ POSITION\n %f %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5]);

	interface.ui_msg->message(buffer);

	return (Pt_CONTINUE);

}

int irp6m_xyz_euler_zyz_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	double *wektor[6];
	double tool_vector[6];

	// wychwytania ew. bledow ECP::robot
try {
	if (interface.irp6m_m->state.edp.is_synchronised) {

		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p1,
				Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p2,
				Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p3,
				Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p4,
				Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p5,
				Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
		PtGetResource(ABW_PtNumericFloat_wind_irp6m_xyz_euler_zyz_ts_p6,
				Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

		for (int i = 0; i < 6; i++) {
			tool_vector[i] = *wektor[i];
		}

		// zlecenie wykonania ruchu
		interface.irp6m_m->ui_ecp_robot->set_tool_xyz_euler_zyz(tool_vector);

	} else {
	}
} // end try
CATCH_SECTION_UI

return (Pt_CONTINUE);

}

int clear_wnd_polycrank_inc_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_inc_open = false;

	return (Pt_CONTINUE);

}

int clear_wnd_polycrank_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_int_open = false;

	return (Pt_CONTINUE);

}

int clear_wnd_polycrank_kinematic_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_kinematic_open = 0;

	return (Pt_CONTINUE);
}

int clear_wnd_polycrank_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_servo_algorithm_open = false;

	return (Pt_CONTINUE);
}

int clear_wnd_polycrank_xyz_angle_axis_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_open = false;

	return (Pt_CONTINUE);

}

int clear_wnd_polycrank_xyz_aa_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_ts_open = 0;

	return (Pt_CONTINUE);

}

int clear_wnd_polycrank_xyz_euler_zyz_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_open = false;

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_inc_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_inc);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_int_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_int);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_xyz_angle_axis);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_xyz_euler_zyz);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_xyz_euler_zyz_ts_open) {
		PtDestroyWidget ( ABW_wnd_polycrank_xyz_euler_zyz_ts);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_kinematic_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_kinematic);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int close_wnd_polycrank_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_ts_open) {
		PtDestroyWidget(ABW_wnd_irp6_mechatronika_xyz_angle_axis_ts);
	}

	return (Pt_CONTINUE);

}

int start_wnd_polycrank_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_polycrank_xyz_angle_axis, widget, cbinfo);
		interface.irp6m_m->is_wind_irp6m_xyz_angle_axis_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6_mechatronika_xyz_angle_axis);
	}

	return (Pt_CONTINUE);

}

// aktualizacja ustawien przyciskow
int process_control_window_polycrank_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (interface.irp6m_m->state.edp.state <= 0) {// edp wylaczone
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_start);
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_stop);
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_trigger);
	} else {
		if (interface.irp6m_m->state.edp.state == 1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start = true;
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_start);
			interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_stop);
			interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_trigger);
		} else if (interface.irp6m_m->state.edp.state == 2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
			interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_start);
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_stop);
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6m_reader_trigger);
		}
	}

	interface.irp6m_m->state.edp.last_state = interface.irp6m_m->state.edp.state;

	return 1;

}

int reload_polycrank_configuration()
{
	// jesli IRP6 mechatronika ma byc aktywne
	if ((interface.irp6m_m->state.is_active = interface.config->value <int> ("is_irp6m_active")) == 1) {

		// ui_state.is_any_edp_active = true;
		// ini_con->create_ecp_irp6_mechatronika (ini_con->ui->ecp_irp6_mechatronika_section);
		if (interface.is_mp_and_ecps_active) {
			interface.irp6m_m->state.ecp.network_trigger_attach_point
					= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point", interface.irp6m_m->state.ecp.section_name.c_str());

			interface.irp6m_m->state.ecp.pid = -1;
			interface.irp6m_m->state.ecp.trigger_fd = common::invalid_fd;
		}

		switch (interface.irp6m_m->state.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6_mechatronika (ini_con->ui->edp_irp6_mechatronika_section);

				interface.irp6m_m->state.edp.pid = -1;
				interface.irp6m_m->state.edp.reader_fd = common::invalid_fd;
				interface.irp6m_m->state.edp.state = 0;

				for (int i = 0; i < 3; i++) {
					char tmp_string[50];
					sprintf(tmp_string, "preset_position_%d", i);

					if (interface.config->exists(tmp_string, interface.irp6m_m->state.edp.section_name)) {
						char* tmp, *tmp1;
						tmp1
								= tmp
										= strdup(interface.config->value <std::string> (tmp_string, interface.irp6m_m->state.edp.section_name).c_str());
						char* toDel = tmp;
						for (int j = 0; j < 8; j++) {
							interface.irp6m_m->state.edp.preset_position[i][j] = strtod(tmp1, &tmp1);
						}
						free(toDel);
					} else {
						for (int j = 0; j < 7; j++) {
							interface.irp6m_m->state.edp.preset_position[i][j] = 0.0;
						}
					}
				}

				if (interface.config->exists(lib::ROBOT_TEST_MODE, interface.irp6m_m->state.edp.section_name))
					interface.irp6m_m->state.edp.test_mode
							= interface.config->value <int> (lib::ROBOT_TEST_MODE, interface.irp6m_m->state.edp.section_name);
				else
					interface.irp6m_m->state.edp.test_mode = 0;

				interface.irp6m_m->state.edp.hardware_busy_attach_point
						= interface.config->value <std::string> ("hardware_busy_attach_point", interface.irp6m_m->state.edp.section_name);

				interface.irp6m_m->state.edp.network_resourceman_attach_point
						= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", interface.irp6m_m->state.edp.section_name.c_str());

				interface.irp6m_m->state.edp.network_reader_attach_point
						= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point", interface.irp6m_m->state.edp.section_name.c_str());

				interface.irp6m_m->state.edp.node_name
						= interface.config->value <std::string> ("node_name", interface.irp6m_m->state.edp.section_name.c_str());

				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}

	} else // jesli  irp6 mechatronika ma byc nieaktywne
	{
		switch (interface.irp6m_m->state.edp.state)
		{
			case -1:
			case 0:
				interface.irp6m_m->state.edp.state = -1;
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}
	} // end irp6_mechatronika

	return 1;
}
