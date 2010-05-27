/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/spkm/ui_r_spkm.h"
#include "lib/robot_consts/spkm_const.h"
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


UiRobotSpkm::UiRobotSpkm() :
	UiRobot(EDP_SPKM_SECTION, ECP_SPKM_SECTION), ui_ecp_robot(NULL) {

}

int UiRobotSpkm::reload_configuration() {
	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = ui.config->value<int> ("is_spkm_active"))
			== 1) {
		// ini_con->create_ecp_spkm (ini_con->ui->ecp_spkm_section);
		//ui_state.is_any_edp_active = true;
		if (ui.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"trigger_attach_point",
							state.ecp.section_name);

			state.ecp.pid = -1;
			state.ecp.trigger_fd = -1;
		}

		switch (state.edp.state) {
		case -1:
		case 0:
			// ini_con->create_edp_spkm (ini_con->ui->edp_spkm_section);

			state.edp.pid = -1;
			state.edp.reader_fd = -1;
			state.edp.state = 0;

			if (ui.config->exists("test_mode", state.edp.section_name))
				state.edp.test_mode = ui.config->value<int> (
						"test_mode", state.edp.section_name);
			else
				state.edp.test_mode = 0;

			state.edp.hardware_busy_attach_point = ui.config->value<
					std::string> ("hardware_busy_attach_point",
					state.edp.section_name);

			state.edp.network_resourceman_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"resourceman_attach_point",
							state.edp.section_name);

			state.edp.network_reader_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"reader_attach_point",
							state.edp.section_name);

			state.edp.node_name = ui.config->value<std::string> (
					"node_name", state.edp.section_name);
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
	} // end spkm

	return 1;
}

int UiRobotSpkm::manage_interface() {
	switch (state.edp.state) {
	case -1:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_spkm, NULL);
		break;
	case 0:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_spkm_edp_unload,

		NULL);
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_spkm,
				ABN_mm_spkm_edp_load, NULL);

		break;
	case 1:
	case 2:
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_spkm, NULL);

		// jesli robot jest zsynchronizowany
		if (state.edp.is_synchronised) {
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);

			switch (ui.mp.state) {
			case UI_MP_NOT_PERMITED_TO_RUN:
			case UI_MP_PERMITED_TO_RUN:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_spkm_edp_unload, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_spkm_edp_load, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

				NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_spkm_edp_load, ABN_mm_spkm_edp_unload, NULL);
				break;
			case UI_MP_TASK_RUNNING:
			case UI_MP_TASK_PAUSED:
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						NULL);
				break;
			default:
				break;
			}
		} else // jesli robot jest niezsynchronizowany
		{
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
					ABN_mm_spkm_edp_unload, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_spkm_edp_load,
					NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_synchronisation, NULL);
		}
		break;
	default:
		break;
	}

	return 1;
}

