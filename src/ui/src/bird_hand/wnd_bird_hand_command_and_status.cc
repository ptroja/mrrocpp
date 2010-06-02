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


WndBirdHandCommandAndStatus::WndBirdHandCommandAndStatus(Ui& _ui,
		UiRobotBirdHand& _bird_hand) :
	ui(_ui), bird_hand(_bird_hand), is_open(false) {

}

int WndBirdHandCommandAndStatus::get_index_f_0_command() {

	unsigned long *flags;

	mrrocpp::lib::bird_hand_command &bhcs =
			bird_hand.ui_ecp_robot->bird_hand_command_structure;

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

int WndBirdHandCommandAndStatus::set_index_f_0_status() {

	mrrocpp::lib::bird_hand_status &bhsrs =
			bird_hand.ui_ecp_robot->bird_hand_status_reply_structure;

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
