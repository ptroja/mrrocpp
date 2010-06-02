/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "ui/src/bird_hand/ui_r_bird_hand.h"
#include "ui/src/bird_hand/wnd_bird_hand_command_and_status.h"
#include "lib/robot_consts/bird_hand_const.h"
#include "ui/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

//
//
// KLASA UiRobotBirdHand
//
//


UiRobotBirdHand::UiRobotBirdHand(Ui& _ui) :
	UiRobot(_ui, EDP_BIRD_HAND_SECTION, ECP_BIRD_HAND_SECTION),
	ui_ecp_robot(NULL), is_wnd_bird_hand_configuration_open(false) {
	wnd_command_and_status = new WndBirdHandCommandAndStatus(ui, *this);

}

int UiRobotBirdHand::reload_configuration() {
	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = ui.config->value<int> ("is_bird_hand_active")) == 1) {
		// ini_con->create_ecp_bird_hand (ini_con->ui->ecp_bird_hand_section);
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
			// ini_con->create_edp_bird_hand (ini_con->ui->edp_bird_hand_section);

			state.edp.pid = -1;
			state.edp.reader_fd = -1;
			state.edp.state = 0;

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
	} // end bird_hand

	return 1;
}

int UiRobotBirdHand::manage_interface() {
	switch (state.edp.state) {
	case -1:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand, NULL);
		break;
	case 0:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM,
				ABN_mm_bird_hand_edp_unload, ABN_mm_bird_hand_command,
				ABN_mm_bird_hand_configuration, NULL);
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand,
				ABN_mm_bird_hand_edp_load, NULL);

		break;
	case 1:
	case 2:
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand, NULL);

		// jesli robot jest zsynchronizowany
		if (state.edp.is_synchronised) {
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);

			switch (ui.mp.state) {
			case UI_MP_NOT_PERMITED_TO_RUN:
			case UI_MP_PERMITED_TO_RUN:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_bird_hand_edp_unload, ABN_mm_bird_hand_command,
						ABN_mm_bird_hand_configuration, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_bird_hand_edp_load, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_bird_hand_command,
						ABN_mm_bird_hand_configuration, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_edp_unload,
						NULL);
				break;
			case UI_MP_TASK_RUNNING:
			case UI_MP_TASK_PAUSED:
				ApModifyItemState(&robot_menu,
						AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_bird_hand_command,
						ABN_mm_bird_hand_configuration, NULL);
				break;
			default:
				break;
			}
		} else // jesli robot jest niezsynchronizowany
		{
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
					ABN_mm_bird_hand_edp_unload, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_DIM,
					ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_command,
					ABN_mm_bird_hand_configuration, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_synchronisation, NULL);
		}
		break;
	default:
		break;
	}

	return 1;
}

int UiRobotBirdHand::close_all_windows() {

	int pt_res = PtEnter(0);

	close_wnd_bird_hand_command_and_status(NULL, NULL, NULL);
	close_wnd_bird_hand_configuration(NULL, NULL, NULL);

	if (pt_res >= 0) {
		PtLeave(0);
	}
	return 1;

}

int UiRobotBirdHand::get_index_f_0_command() {

	unsigned long *flags;

	mrrocpp::lib::bird_hand_command &bhcs =
			ui_ecp_robot->bird_hand_command_structure;

	PtGetResource(
			ABW_index_f_0_absolute_variant_wnd_bird_hand_command_and_status,
			Pt_ARG_FLAGS, &flags, 0);

	if (*flags & Pt_SET) {
		bhcs.index_f[0].profile_type
				= lib::BIRD_HAND_MACROSTEP_ABSOLUTE_POSITION;
	}

	PtGetResource(
			ABW_index_f_0_relative_variant_wnd_bird_hand_command_and_status,
			Pt_ARG_FLAGS, &flags, 0);

	if (*flags & Pt_SET) {
		bhcs.index_f[0].profile_type
				= lib::BIRD_HAND_MACROSTEP_POSITION_INCREMENT;
	}

	PtGetResource(
			ABW_index_f_0_velocity_variant_wnd_bird_hand_command_and_status,
			Pt_ARG_FLAGS, &flags, 0);

	if (*flags & Pt_SET) {
		bhcs.index_f[0].profile_type
				= lib::BIRD_HAND_SIGLE_STEP_POSTION_INCREMENT;
	}

	double* tmp_double;

	PtGetResource(
			ABW_index_f_0_desired_position_wnd_bird_hand_command_and_status,
			Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

	bhcs.index_f[0].desired_position = *tmp_double;

	PtGetResource(
			ABW_index_f_0_desired_torque_wnd_bird_hand_command_and_status,
			Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

	bhcs.index_f[0].desired_torque = *tmp_double;

	PtGetResource(
			ABW_index_f_0_recip_of_damping_wnd_bird_hand_command_and_status,
			Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

	bhcs.index_f[0].reciprocal_of_damping = *tmp_double;

	return 1;

}

int UiRobotBirdHand::set_index_f_0_status() {

	mrrocpp::lib::bird_hand_status &bhsrs =
			ui_ecp_robot->bird_hand_status_reply_structure;

	PtSetResource(
			ABW_index_f_0_current_position_wnd_bird_hand_command_and_status,
			Pt_ARG_NUMERIC_VALUE, &bhsrs.index_f[0].meassured_position, 0);
	PtSetResource(
			ABW_index_f_0_current_torque_wnd_bird_hand_command_and_status,
			Pt_ARG_NUMERIC_VALUE, &bhsrs.index_f[0].meassured_torque, 0);
	PtSetResource(
			ABW_index_f_0_meassured_current_wnd_bird_hand_command_and_status,
			Pt_ARG_NUMERIC_VALUE, &bhsrs.index_f[0].meassured_current, 0);

	if (bhsrs.index_f[0].lower_limit_of_absolute_position) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_1_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_1_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].lower_limit_of_absolute_value_of_desired_torque) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_2_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_2_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].upper_limit_of_absolute_position) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_3_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_3_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].upper_limit_of_absolute_value_of_computed_position_increment) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_4_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_4_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].upper_limit_of_absolute_value_of_desired_position_increment) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_5_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_5_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].upper_limit_of_absolute_value_of_desired_torque) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_6_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_6_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].upper_limit_of_absolute_value_of_meassured_torque) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_7_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_7_wnd_bird_hand_command_and_status);
	}

	if (bhsrs.index_f[0].upper_limit_of_meassured_current) {
		ui.set_toggle_button(
				ABW_index_f_0_limit_8_wnd_bird_hand_command_and_status);

	} else {
		ui.unset_toggle_button(
				ABW_index_f_0_limit_8_wnd_bird_hand_command_and_status);
	}

	return 1;

}

int UiRobotBirdHand::delete_ui_ecp_robot() {
	delete ui_ecp_robot;
	return 1;
}

