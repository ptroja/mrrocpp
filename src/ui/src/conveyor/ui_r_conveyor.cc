/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/conveyor/ui_r_conveyor.h"
#include "lib/robot_consts/conveyor_const.h"
#include "ui/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

extern Ui ui;

// extern ui_state_def ui_state;

//
//
// KLASA UiRobotIrp6ot_m
//
//


UiRobotConveyor::UiRobotConveyor() :
	UiRobot(EDP_CONVEYOR_SECTION, ECP_CONVEYOR_SECTION), ui_ecp_robot(NULL),
			is_wind_conv_servo_algorithm_open(false),
			is_wind_conveyor_moves_open(false) {

}

int UiRobotConveyor::reload_configuration() {

	// jesli conveyor ma byc aktywny
	if ((state.is_active = ui.config->value<int> ("is_conveyor_active")) == 1) {

		//ui_state.is_any_edp_active = true;

		if (ui.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"trigger_attach_point",
							state.ecp.section_name.c_str());

			state.ecp.pid = -1;
			state.ecp.trigger_fd = -1;
		}

		switch (state.edp.state) {
		case -1:
		case 0:

			state.edp.pid = -1;
			state.edp.reader_fd = -1;
			state.edp.state = 0;

			if (ui.config->exists("preset_position_0", state.edp.section_name))
				state.edp.preset_position[0][0] = ui.config->value<double> (
						"preset_position_0", state.edp.section_name);
			if (ui.config->exists("preset_position_1", state.edp.section_name))
				state.edp.preset_position[1][0] = ui.config->value<double> (
						"preset_position_1", state.edp.section_name);
			if (ui.config->exists("preset_position_2", state.edp.section_name))
				state.edp.preset_position[2][0] = ui.config->value<double> (
						"preset_position_2", state.edp.section_name);

			if (ui.config->exists("test_mode", state.edp.section_name))
				state.edp.test_mode = ui.config->value<int> ("test_mode",
						state.edp.section_name);
			else
				state.edp.test_mode = 0;

			state.edp.hardware_busy_attach_point
					= ui.config->value<std::string> (
							"hardware_busy_attach_point",
							state.edp.section_name);

			state.edp.network_resourceman_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"resourceman_attach_point",
							state.edp.section_name.c_str());

			state.edp.network_reader_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"reader_attach_point",
							state.edp.section_name.c_str());

			state.edp.node_name = ui.config->value<std::string> ("node_name",
					state.edp.section_name.c_str());

			break;
		case 1:
		case 2:
			// nie robi nic bo EDP pracuje
			break;
		default:
			break;
		}

	} else // jesli  conveyor ma byc nieaktywny
	{

		switch (state.edp.state) {
		case -1:
		case 0:
			state.edp.state = -1;
			break;
		case 1:
		case 2:
			// nie robi nic bo EDP pracuje
			break;
		default:
			break;
		}
	} // end conveyor

	return 1;
}

int UiRobotConveyor::manage_interface() {

	switch (state.edp.state) {
	case -1:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_conveyor, NULL);
		break;
	case 0:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_conveyor_edp_unload,
				ABN_mm_conveyor_synchronisation, ABN_mm_conveyor_move,
				ABN_mm_conveyor_preset_positions,
				ABN_mm_conveyor_servo_algorithm, NULL);
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_conveyor,
				ABN_mm_conveyor_edp_load, NULL);

		break;
	case 1:
	case 2:
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_conveyor, NULL);

		// jesli robot jest zsynchronizowany
		if (state.edp.is_synchronised) {
			ApModifyItemState(&robot_menu, AB_ITEM_DIM,
					ABN_mm_conveyor_synchronisation, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);

			switch (ui.mp.state) {
			case UI_MP_NOT_PERMITED_TO_RUN:
			case UI_MP_PERMITED_TO_RUN:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_conveyor_edp_unload, ABN_mm_conveyor_move,
						ABN_mm_conveyor_preset_positions,
						ABN_mm_conveyor_servo_algorithm, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_conveyor_edp_load, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_conveyor_move, ABN_mm_conveyor_preset_positions,
						ABN_mm_conveyor_servo_algorithm, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_conveyor_edp_load, ABN_mm_conveyor_edp_unload,
						NULL);
				break;
			case UI_MP_TASK_RUNNING:
			case UI_MP_TASK_PAUSED:
				ApModifyItemState(&robot_menu,
						AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_conveyor_move, ABN_mm_conveyor_preset_positions,
						ABN_mm_conveyor_servo_algorithm, NULL);
				break;
			default:
				break;
			}
		} else // jesli robot jest niezsynchronizowany
		{
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
					ABN_mm_conveyor_edp_unload,
					ABN_mm_conveyor_synchronisation, ABN_mm_conveyor_move, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_DIM,
					ABN_mm_conveyor_edp_load, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_synchronisation, NULL);
		}
		break;
	default:
		break;
	}

	return 1;
}

// aktualizacja ustawien przyciskow
int UiRobotConveyor::process_control_window_conveyor_section_init(
		bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
		bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
		bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger) {

	if (state.edp.state <= 0) {// edp wylaczone
		block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_start);
		block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_stop);
		block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_trigger);
	} else {
		if (state.edp.state == 1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start = true;
			unblock_widget(
					ABW_PtButton_wnd_processes_control_conveyor_reader_start);
			block_widget(
					ABW_PtButton_wnd_processes_control_conveyor_reader_stop);
			block_widget(
					ABW_PtButton_wnd_processes_control_conveyor_reader_trigger);
		} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
			block_widget(
					ABW_PtButton_wnd_processes_control_conveyor_reader_start);
			unblock_widget(
					ABW_PtButton_wnd_processes_control_conveyor_reader_stop);
			unblock_widget(
					ABW_PtButton_wnd_processes_control_conveyor_reader_trigger);
		}
	}

	state.edp.last_state = state.edp.state;

	return 1;

}
