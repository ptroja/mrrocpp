/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "ui/src/bird_hand/ui_r_bird_hand.h"
#include "ui/src/bird_hand/wnd_bird_hand_command_and_status.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace bird_hand {

//
//
// KLASA UiRobotBirdHand
//
//


WndCommandAndStatus::WndCommandAndStatus(common::Interface& _interface, UiRobot& _bird_hand) :
			common::WndBase(WND_BIRD_HAND_COMMAND_AND_STATUS, _interface, ABN_wnd_bird_hand_command_and_status, ABI_wnd_bird_hand_command_and_status),
			bird_hand(_bird_hand)
{

}

int WndCommandAndStatus::get_command()
{

	try {

		mrrocpp::lib::bird_hand::command &bhcs = bird_hand.ui_ecp_robot->bird_hand_command_data_port->data;

		// odczyt ilosci krokow i ecp_query step

		int* motion_steps, *ecp_query_step;

		PtGetResource(ABW_motion_steps_wnd_bird_hand_command_and_status, Pt_ARG_NUMERIC_VALUE, &motion_steps, 0);
		PtGetResource(ABW_ecp_query_step_wnd_bird_hand_command_and_status, Pt_ARG_NUMERIC_VALUE, &ecp_query_step, 0);

		bhcs.motion_steps = *motion_steps;
		bhcs.ecp_query_step = *ecp_query_step;

		get_finger_command(bhcs.thumb_f[0], ABW_thumb_f_0_desired_position_wnd_bird_hand_command_and_status, ABW_thumb_f_0_desired_torque_wnd_bird_hand_command_and_status, ABW_thumb_f_0_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.thumb_f[0], ABW_thumb_f_0_absolute_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_0_relative_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_0_velocity_variant_wnd_bird_hand_command_and_status);
		get_finger_command(bhcs.thumb_f[1], ABW_thumb_f_1_desired_position_wnd_bird_hand_command_and_status, ABW_thumb_f_1_desired_torque_wnd_bird_hand_command_and_status, ABW_thumb_f_1_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.thumb_f[1], ABW_thumb_f_1_absolute_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_1_relative_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_1_velocity_variant_wnd_bird_hand_command_and_status);

		get_finger_command(bhcs.index_f[0], ABW_index_f_0_desired_position_wnd_bird_hand_command_and_status, ABW_index_f_0_desired_torque_wnd_bird_hand_command_and_status, ABW_index_f_0_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.index_f[0], ABW_index_f_0_absolute_variant_wnd_bird_hand_command_and_status, ABW_index_f_0_relative_variant_wnd_bird_hand_command_and_status, ABW_index_f_0_velocity_variant_wnd_bird_hand_command_and_status);
		get_finger_command(bhcs.index_f[1], ABW_index_f_1_desired_position_wnd_bird_hand_command_and_status, ABW_index_f_1_desired_torque_wnd_bird_hand_command_and_status, ABW_index_f_1_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.index_f[1], ABW_index_f_1_absolute_variant_wnd_bird_hand_command_and_status, ABW_index_f_1_relative_variant_wnd_bird_hand_command_and_status, ABW_index_f_1_velocity_variant_wnd_bird_hand_command_and_status);
		get_finger_command(bhcs.index_f[2], ABW_index_f_2_desired_position_wnd_bird_hand_command_and_status, ABW_index_f_2_desired_torque_wnd_bird_hand_command_and_status, ABW_index_f_2_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.index_f[2], ABW_index_f_2_absolute_variant_wnd_bird_hand_command_and_status, ABW_index_f_2_relative_variant_wnd_bird_hand_command_and_status, ABW_index_f_2_velocity_variant_wnd_bird_hand_command_and_status);

		get_finger_command(bhcs.ring_f[0], ABW_ring_f_0_desired_position_wnd_bird_hand_command_and_status, ABW_ring_f_0_desired_torque_wnd_bird_hand_command_and_status, ABW_ring_f_0_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.ring_f[0], ABW_ring_f_0_absolute_variant_wnd_bird_hand_command_and_status, ABW_ring_f_0_relative_variant_wnd_bird_hand_command_and_status, ABW_ring_f_0_velocity_variant_wnd_bird_hand_command_and_status);
		get_finger_command(bhcs.ring_f[1], ABW_ring_f_1_desired_position_wnd_bird_hand_command_and_status, ABW_ring_f_1_desired_torque_wnd_bird_hand_command_and_status, ABW_ring_f_1_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.ring_f[1], ABW_ring_f_1_absolute_variant_wnd_bird_hand_command_and_status, ABW_ring_f_1_relative_variant_wnd_bird_hand_command_and_status, ABW_ring_f_1_velocity_variant_wnd_bird_hand_command_and_status);
		get_finger_command(bhcs.ring_f[2], ABW_ring_f_2_desired_position_wnd_bird_hand_command_and_status, ABW_ring_f_2_desired_torque_wnd_bird_hand_command_and_status, ABW_ring_f_2_recip_of_damping_wnd_bird_hand_command_and_status);
		get_variant_finger_command(bhcs.ring_f[2], ABW_ring_f_2_absolute_variant_wnd_bird_hand_command_and_status, ABW_ring_f_2_relative_variant_wnd_bird_hand_command_and_status, ABW_ring_f_2_velocity_variant_wnd_bird_hand_command_and_status);

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		/*
		 ss << bhcs.index_f[0].profile_type << " " << bhcs.motion_steps << "  "
		 << bhcs.ecp_query_step;
		 */
		/*
		 ss << bhcs.index_f[0].desired_position << " "
		 << bhcs.index_f[0].desired_torque << "  "
		 << bhcs.index_f[0].reciprocal_of_damping;

		 interface.ui_msg->message(ss.str().c_str());
		 */
		bird_hand.ui_ecp_robot->bird_hand_command_data_port->set();
		bird_hand.ui_ecp_robot->execute_motion();

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int WndCommandAndStatus::set_status()
{

	try {

		mrrocpp::lib::bird_hand::status &bhsrs = bird_hand.ui_ecp_robot->bird_hand_status_reply_data_request_port->data;

		bird_hand.ui_ecp_robot->bird_hand_status_reply_data_request_port->set_request();
		bird_hand.ui_ecp_robot->execute_motion();
		bird_hand.ui_ecp_robot->bird_hand_status_reply_data_request_port->get();

		set_finger_status(bhsrs.thumb_f[0], ABW_thumb_f_0_current_position_wnd_bird_hand_command_and_status, ABW_thumb_f_0_current_torque_wnd_bird_hand_command_and_status, ABW_thumb_f_0_meassured_current_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_1_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_2_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_3_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_4_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_5_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_6_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_7_wnd_bird_hand_command_and_status, ABW_thumb_f_0_limit_8_wnd_bird_hand_command_and_status);
		set_finger_status(bhsrs.thumb_f[1], ABW_thumb_f_1_current_position_wnd_bird_hand_command_and_status, ABW_thumb_f_1_current_torque_wnd_bird_hand_command_and_status, ABW_thumb_f_1_meassured_current_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_1_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_2_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_3_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_4_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_5_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_6_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_7_wnd_bird_hand_command_and_status, ABW_thumb_f_1_limit_8_wnd_bird_hand_command_and_status);

		set_finger_status(bhsrs.index_f[0], ABW_index_f_0_current_position_wnd_bird_hand_command_and_status, ABW_index_f_0_current_torque_wnd_bird_hand_command_and_status, ABW_index_f_0_meassured_current_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_1_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_2_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_3_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_4_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_5_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_6_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_7_wnd_bird_hand_command_and_status, ABW_index_f_0_limit_8_wnd_bird_hand_command_and_status);
		set_finger_status(bhsrs.index_f[1], ABW_index_f_1_current_position_wnd_bird_hand_command_and_status, ABW_index_f_1_current_torque_wnd_bird_hand_command_and_status, ABW_index_f_1_meassured_current_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_1_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_2_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_3_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_4_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_5_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_6_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_7_wnd_bird_hand_command_and_status, ABW_index_f_1_limit_8_wnd_bird_hand_command_and_status);
		set_finger_status(bhsrs.index_f[2], ABW_index_f_2_current_position_wnd_bird_hand_command_and_status, ABW_index_f_2_current_torque_wnd_bird_hand_command_and_status, ABW_index_f_2_meassured_current_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_2_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_2_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_3_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_4_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_5_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_6_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_7_wnd_bird_hand_command_and_status, ABW_index_f_2_limit_8_wnd_bird_hand_command_and_status);

		set_finger_status(bhsrs.ring_f[0], ABW_ring_f_0_current_position_wnd_bird_hand_command_and_status, ABW_ring_f_0_current_torque_wnd_bird_hand_command_and_status, ABW_ring_f_0_meassured_current_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_1_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_2_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_3_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_4_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_5_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_6_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_7_wnd_bird_hand_command_and_status, ABW_ring_f_0_limit_8_wnd_bird_hand_command_and_status);
		set_finger_status(bhsrs.ring_f[1], ABW_ring_f_1_current_position_wnd_bird_hand_command_and_status, ABW_ring_f_1_current_torque_wnd_bird_hand_command_and_status, ABW_ring_f_1_meassured_current_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_1_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_2_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_3_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_4_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_5_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_6_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_7_wnd_bird_hand_command_and_status, ABW_ring_f_1_limit_8_wnd_bird_hand_command_and_status);
		set_finger_status(bhsrs.ring_f[2], ABW_ring_f_2_current_position_wnd_bird_hand_command_and_status, ABW_ring_f_2_current_torque_wnd_bird_hand_command_and_status, ABW_ring_f_2_meassured_current_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_2_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_2_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_3_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_4_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_5_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_6_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_7_wnd_bird_hand_command_and_status, ABW_ring_f_2_limit_8_wnd_bird_hand_command_and_status);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int WndCommandAndStatus::copy_command()
{
	mrrocpp::lib::bird_hand::command &bhcs = bird_hand.ui_ecp_robot->bird_hand_command_data_port->data;

	get_variant_finger_command(bhcs.thumb_f[0], ABW_thumb_f_0_absolute_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_0_relative_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_0_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.thumb_f[0], ABW_thumb_f_0_current_position_wnd_bird_hand_command_and_status, ABW_thumb_f_0_desired_position_wnd_bird_hand_command_and_status);
	get_variant_finger_command(bhcs.thumb_f[1], ABW_thumb_f_1_absolute_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_1_relative_variant_wnd_bird_hand_command_and_status, ABW_thumb_f_1_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.thumb_f[1], ABW_thumb_f_1_current_position_wnd_bird_hand_command_and_status, ABW_thumb_f_1_desired_position_wnd_bird_hand_command_and_status);

	get_variant_finger_command(bhcs.index_f[0], ABW_index_f_0_absolute_variant_wnd_bird_hand_command_and_status, ABW_index_f_0_relative_variant_wnd_bird_hand_command_and_status, ABW_index_f_0_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.index_f[0], ABW_index_f_0_current_position_wnd_bird_hand_command_and_status, ABW_index_f_0_desired_position_wnd_bird_hand_command_and_status);
	get_variant_finger_command(bhcs.index_f[1], ABW_index_f_1_absolute_variant_wnd_bird_hand_command_and_status, ABW_index_f_1_relative_variant_wnd_bird_hand_command_and_status, ABW_index_f_1_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.index_f[1], ABW_index_f_1_current_position_wnd_bird_hand_command_and_status, ABW_index_f_1_desired_position_wnd_bird_hand_command_and_status);
	get_variant_finger_command(bhcs.index_f[2], ABW_index_f_2_absolute_variant_wnd_bird_hand_command_and_status, ABW_index_f_2_relative_variant_wnd_bird_hand_command_and_status, ABW_index_f_2_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.index_f[2], ABW_index_f_2_current_position_wnd_bird_hand_command_and_status, ABW_index_f_2_desired_position_wnd_bird_hand_command_and_status);

	get_variant_finger_command(bhcs.ring_f[0], ABW_ring_f_0_absolute_variant_wnd_bird_hand_command_and_status, ABW_ring_f_0_relative_variant_wnd_bird_hand_command_and_status, ABW_ring_f_0_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.ring_f[0], ABW_ring_f_0_current_position_wnd_bird_hand_command_and_status, ABW_ring_f_0_desired_position_wnd_bird_hand_command_and_status);
	get_variant_finger_command(bhcs.ring_f[1], ABW_ring_f_1_absolute_variant_wnd_bird_hand_command_and_status, ABW_ring_f_1_relative_variant_wnd_bird_hand_command_and_status, ABW_ring_f_1_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.ring_f[1], ABW_ring_f_1_current_position_wnd_bird_hand_command_and_status, ABW_ring_f_1_desired_position_wnd_bird_hand_command_and_status);
	get_variant_finger_command(bhcs.ring_f[2], ABW_ring_f_2_absolute_variant_wnd_bird_hand_command_and_status, ABW_ring_f_2_relative_variant_wnd_bird_hand_command_and_status, ABW_ring_f_2_velocity_variant_wnd_bird_hand_command_and_status);
	copy_finger_command(bhcs.ring_f[2], ABW_ring_f_2_current_position_wnd_bird_hand_command_and_status, ABW_ring_f_2_desired_position_wnd_bird_hand_command_and_status);

	return 1;
}

int WndCommandAndStatus::get_variant_finger_command(lib::bird_hand::single_joint_command &finger, PtWidget_t *ABW_absolute, PtWidget_t *ABW_relative, PtWidget_t *ABW_velocity)
{

	unsigned long *flags;

	PtGetResource(ABW_absolute, Pt_ARG_FLAGS, &flags, 0);

	if (*flags & Pt_SET) {
		finger.profile_type = lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION;
	}

	PtGetResource(ABW_relative, Pt_ARG_FLAGS, &flags, 0);

	if (*flags & Pt_SET) {
		finger.profile_type = lib::bird_hand::MACROSTEP_POSITION_INCREMENT;
	}

	PtGetResource(ABW_velocity, Pt_ARG_FLAGS, &flags, 0);

	if (*flags & Pt_SET) {
		finger.profile_type = lib::bird_hand::SIGLE_STEP_POSTION_INCREMENT;
	}

	return 1;

}

int WndCommandAndStatus::get_finger_command(lib::bird_hand::single_joint_command &finger, PtWidget_t *ABW_position, PtWidget_t *ABW_torque, PtWidget_t *ABW_damping)
{

	double* tmp_double;

	PtGetResource(ABW_position, Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

	finger.desired_position = *tmp_double;

	PtGetResource(ABW_torque, Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

	finger.desired_torque = *tmp_double;

	PtGetResource(ABW_damping, Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

	finger.reciprocal_of_damping = *tmp_double;

	return 1;

}

int WndCommandAndStatus::set_finger_status(lib::bird_hand::single_joint_status &finger, PtWidget_t *ABW_position, PtWidget_t *ABW_torque, PtWidget_t *ABW_current, PtWidget_t *ABW_limit_1, PtWidget_t *ABW_limit_2, PtWidget_t *ABW_limit_3, PtWidget_t *ABW_limit_4, PtWidget_t *ABW_limit_5, PtWidget_t *ABW_limit_6, PtWidget_t *ABW_limit_7, PtWidget_t *ABW_limit_8)
{

	PtSetResource(ABW_position, Pt_ARG_NUMERIC_VALUE, &finger.meassured_position, 0);
	PtSetResource(ABW_torque, Pt_ARG_NUMERIC_VALUE, &finger.meassured_torque, 0);
	PtSetResource(ABW_current, Pt_ARG_NUMERIC_VALUE, &finger.meassured_current, 0);

	if (finger.lower_limit_of_absolute_position) {
		interface.set_toggle_button(ABW_limit_1);

	} else {
		interface.unset_toggle_button(ABW_limit_1);
	}

	if (finger.lower_limit_of_absolute_value_of_desired_torque) {
		interface.set_toggle_button(ABW_limit_2);
	} else {
		interface.unset_toggle_button(ABW_limit_2);
	}

	if (finger.upper_limit_of_absolute_position) {
		interface.set_toggle_button(ABW_limit_3);
	} else {
		interface.unset_toggle_button(ABW_limit_3);
	}

	if (finger.upper_limit_of_absolute_value_of_computed_position_increment) {
		interface.set_toggle_button(ABW_limit_4);
	} else {
		interface.unset_toggle_button(ABW_limit_4);
	}

	if (finger.upper_limit_of_absolute_value_of_desired_position_increment) {
		interface.set_toggle_button(ABW_limit_5);
	} else {
		interface.unset_toggle_button(ABW_limit_5);
	}

	if (finger.upper_limit_of_absolute_value_of_desired_torque) {
		interface.set_toggle_button(ABW_limit_6);
	} else {
		interface.unset_toggle_button(ABW_limit_6);
	}

	if (finger.upper_limit_of_absolute_value_of_meassured_torque) {
		interface.set_toggle_button(ABW_limit_7);
	} else {
		interface.unset_toggle_button(ABW_limit_7);
	}

	if (finger.upper_limit_of_meassured_current) {
		interface.set_toggle_button(ABW_limit_8);
	} else {
		interface.unset_toggle_button(ABW_limit_8);
	}

	return 1;

}

int WndCommandAndStatus::copy_finger_command(lib::bird_hand::single_joint_command &finger, PtWidget_t *ABW_current, PtWidget_t *ABW_desired)
{

	double* tmp_double;

	if (finger.profile_type == lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION) {

		PtGetResource(ABW_current, Pt_ARG_NUMERIC_VALUE, &tmp_double, 0);

		double set_double = *tmp_double;

		PtSetResource(ABW_desired, Pt_ARG_NUMERIC_VALUE, &set_double, 0);

	}
	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
