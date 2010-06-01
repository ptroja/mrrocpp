/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/irp6ot_tfg/ui_r_irp6ot_tfg.h"
#include "ui/ui_ecp_r_tfg_and_conv.h"
#include "lib/robot_consts/irp6ot_tfg_const.h"
#include "ui/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

//
//
// KLASA UiRobotIrp6ot_tfg
//
//


UiRobotIrp6ot_tfg::UiRobotIrp6ot_tfg(Ui& _ui) :
			UiRobot(_ui, EDP_IRP6OT_TFG_SECTION, ECP_IRP6OT_TFG_SECTION), is_wind_irp6ot_tfg_moves_open(
					false), is_wind_irp6ot_tfg_servo_algorithm_open(false),
			ui_ecp_robot(NULL) {

}

int UiRobotIrp6ot_tfg::reload_configuration() {

	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = ui.config->value<int> ("is_irp6ot_tfg_active")) == 1) {
		// ini_con->create_ecp_irp6ot_tfg (ini_con->ui->ecp_irp6ot_tfg_section);
		//ui_state.is_any_edp_active = true;
		if (ui.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"trigger_attach_point", state.ecp.section_name);

			state.ecp.pid = -1;
			state.ecp.trigger_fd = -1;
		}

		switch (state.edp.state) {
		case -1:
		case 0:
			// ini_con->create_edp_irp6ot_tfg (ini_con->ui->edp_irp6ot_tfg_section);

			state.edp.pid = -1;
			state.edp.reader_fd = -1;
			state.edp.state = 0;

			for (int i = 0; i < 3; i++) {
				char tmp_string[50];
				sprintf(tmp_string, "preset_position_%d", i);

				if (ui.config->exists(tmp_string, state.edp.section_name)) {
					char* tmp, *tmp1;
					tmp1 = tmp = strdup(ui.config->value<std::string> (
							tmp_string, state.edp.section_name).c_str());
					char* toDel = tmp;
					for (int j = 0; j < IRP6OT_TFG_NUM_OF_SERVOS; j++) {

						state.edp.preset_position[i][j] = strtod(tmp1, &tmp1);

					}
					free(toDel);
				} else {
					for (int j = 0; j < IRP6OT_TFG_NUM_OF_SERVOS; j++) {

						state.edp.preset_position[i][j] = 0.074;

					}
				}
			}

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
							"resourceman_attach_point", state.edp.section_name);

			state.edp.network_reader_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"reader_attach_point", state.edp.section_name);

			state.edp.node_name = ui.config->value<std::string> ("node_name",
					state.edp.section_name);
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
	} // end irp6ot_tfg

	return 1;
}

int UiRobotIrp6ot_tfg::manage_interface() {

	switch (state.edp.state) {
	case -1:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6ot_tfg, NULL);
		break;
	case 0:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM,
				ABN_mm_irp6ot_tfg_edp_unload,
				ABN_mm_irp6ot_tfg_synchronisation, ABN_mm_irp6ot_tfg_move,
				ABN_mm_irp6ot_tfg_preset_positions,
				ABN_mm_irp6ot_tfg_servo_algorithm, NULL);
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6ot_tfg,
				ABN_mm_irp6ot_tfg_edp_load, NULL);

		break;
	case 1:
	case 2:
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6ot_tfg, NULL);

		// jesli robot jest zsynchronizowany
		if (state.edp.is_synchronised) {
			ApModifyItemState(&robot_menu, AB_ITEM_DIM,
					ABN_mm_irp6ot_tfg_synchronisation, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);

			switch (ui.mp.state) {
			case UI_MP_NOT_PERMITED_TO_RUN:
			case UI_MP_PERMITED_TO_RUN:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_irp6ot_tfg_edp_unload, ABN_mm_irp6ot_tfg_move,
						ABN_mm_irp6ot_tfg_preset_positions,
						ABN_mm_irp6ot_tfg_servo_algorithm, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_irp6ot_tfg_edp_load, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_irp6ot_tfg_move,
						ABN_mm_irp6ot_tfg_preset_positions,
						ABN_mm_irp6ot_tfg_servo_algorithm, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_irp6ot_tfg_edp_load,
						ABN_mm_irp6ot_tfg_edp_unload, NULL);
				break;
			case UI_MP_TASK_RUNNING:
			case UI_MP_TASK_PAUSED:
				ApModifyItemState(
						&robot_menu,
						AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_irp6ot_tfg_move,
						ABN_mm_irp6ot_tfg_preset_positions,
						ABN_mm_irp6ot_tfg_servo_algorithm, NULL);
				break;
			default:
				break;
			}
		} else // jesli robot jest niezsynchronizowany
		{
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
					ABN_mm_irp6ot_tfg_edp_unload,
					ABN_mm_irp6ot_tfg_synchronisation, ABN_mm_irp6ot_tfg_move,
					NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_DIM,
					ABN_mm_irp6ot_tfg_edp_load, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_synchronisation, NULL);
		}
		break;
	default:
		break;
	}

	return 1;
}

int UiRobotIrp6ot_tfg::close_all_windows() {

	int pt_res = PtEnter(0);

	close_wind_irp6ot_tfg_moves(NULL, NULL, NULL);
	close_wnd_irp6ot_tfg_servo_algorithm(NULL, NULL, NULL);

	if (pt_res >= 0) {
		PtLeave(0);
	}
	return 1;

}

int UiRobotIrp6ot_tfg::delete_ui_ecp_robot() {
	delete ui_ecp_robot;
	return 1;
}
