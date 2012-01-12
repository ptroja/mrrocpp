/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "base/lib/impconst.h"

#include "robot/smb/ecp_r_smb.h"
#include "base/lib/sr/sr_ecp.h"
#include "robot/smb/kinematic_model_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {

robot::robot(const lib::robot_name_t & _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::_ecp_robot <lib::smb::c_buffer, lib::smb::r_buffer>(_robot_name, lib::smb::NUM_OF_SERVOS, _config, _sr_ecp),
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager),
		epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager),
		epos_external_command_data_port(lib::smb::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager),
		epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager),
		epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager),
		smb_festo_command_data_port(lib::smb::FESTO_COMMAND_DATA_PORT, port_manager),
		epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager),
		epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager),
		epos_external_reply_data_request_port(lib::smb::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager),
		smb_multi_leg_reply_data_request_port(lib::smb::MULTI_LEG_REPLY_DATA_REQUEST_PORT, port_manager)
{

	create_kinematic_models_for_given_robot();
}

robot::robot(const lib::robot_name_t & _robot_name, common::task::task_base& _ecp_object) :
		ecp::common::robot::_ecp_robot <lib::smb::c_buffer, lib::smb::r_buffer>(_robot_name, lib::smb::NUM_OF_SERVOS, _ecp_object),
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager),
		epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager),
		epos_external_command_data_port(lib::smb::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager),
		epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager),
		epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager),
		smb_festo_command_data_port(lib::smb::FESTO_COMMAND_DATA_PORT, port_manager),
		epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager),
		epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager),
		epos_external_reply_data_request_port(lib::smb::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager),
		smb_multi_leg_reply_data_request_port(lib::smb::MULTI_LEG_REPLY_DATA_REQUEST_PORT, port_manager)
{

	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::smb::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

void robot::create_command()
{
	// checks if any data_port is set
	bool is_new_data = false;

	// cheks if any data_request_port is set
	bool is_new_request = false;

	if (epos_motor_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}

		ecp_command.smb.variant = lib::smb::POSE;

		ecp_command.smb.set_pose_specification = lib::smb::MOTOR;

		for (int i = 0; i < lib::smb::NUM_OF_SERVOS; ++i) {
			ecp_command.smb.motor_pos[i] = epos_motor_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_joint_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.smb.variant = lib::smb::POSE;

		ecp_command.smb.set_pose_specification = lib::smb::JOINT;

		for (int i = 0; i < lib::smb::NUM_OF_SERVOS; ++i) {
			ecp_command.smb.joint_pos[i] = epos_joint_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_external_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.smb.variant = lib::smb::POSE;

		ecp_command.smb.set_pose_specification = lib::smb::EXTERNAL;

		ecp_command.smb.base_vs_bench_rotation = epos_external_command_data_port.data.base_vs_bench_rotation;
		ecp_command.smb.pkm_vs_base_rotation = epos_external_command_data_port.data.pkm_vs_base_rotation;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_brake_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}
		// generator command interpretation
		// narazie proste przepisanie

		ecp_command.smb.variant = lib::smb::QUICKSTOP;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_clear_fault_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}
		ecp_command.smb.variant = lib::smb::CLEAR_FAULT;

		check_then_set_command_flag(is_new_data);
	}

	if (smb_festo_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie

		ecp_command.smb.variant = lib::smb::FESTO;

		ecp_command.smb.festo_command = smb_festo_command_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_motor_reply_data_request_port.is_new_request()) {
		ecp_command.smb.get_pose_specification = lib::smb::MOTOR;
		//	ecp_command.get_arm_type = lib::MOTOR;
		//sr_ecp_msg.message("epos_motor_reply_data_request_port");

		check_then_set_command_flag(is_new_request);
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		ecp_command.smb.get_pose_specification = lib::smb::JOINT;
		//ecp_command.get_arm_type = lib::JOINT;
		//	sr_ecp_msg.message("epos_joint_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		ecp_command.smb.get_pose_specification = lib::smb::EXTERNAL;
		//ecp_command.get_arm_type = lib::FRAME;
		//sr_ecp_msg.message("epos_external_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	is_new_request = is_new_request || smb_multi_leg_reply_data_request_port.is_new_request();

	communicate_with_edp = true;

	if (is_new_data && is_new_request) {
		ecp_command.instruction_type = lib::SET_GET;
	} else if (is_new_data) {
		ecp_command.instruction_type = lib::SET;
	} else if (is_new_request) {
		ecp_command.instruction_type = lib::GET;
	} else {
		communicate_with_edp = false;
	}

	if (is_new_request) {
		ecp_command.get_type = ARM_DEFINITION;
	}

}

void robot::get_reply()
{

	if (epos_motor_reply_data_request_port.is_new_request()) {
		// generator reply generation
		for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
			epos_motor_reply_data_request_port.data.epos_controller[i].position =
					reply_package.smb.epos_controller[i].position;
			epos_motor_reply_data_request_port.data.epos_controller[i].current =
					reply_package.smb.epos_controller[i].current;
			epos_motor_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					reply_package.smb.epos_controller[i].motion_in_progress;
		}
		epos_motor_reply_data_request_port.set();
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		// generator reply generation
		sr_ecp_msg.message("ECP get_reply epos_joint_reply_data_request_port");

		for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
			epos_joint_reply_data_request_port.data.epos_controller[i].position =
					reply_package.smb.epos_controller[i].position;
			epos_joint_reply_data_request_port.data.epos_controller[i].current =
					reply_package.smb.epos_controller[i].current;
			epos_joint_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					reply_package.smb.epos_controller[i].motion_in_progress;
		}
		//	epos_joint_reply_data_request_port.data.contact = reply_package.smb.contact;

		epos_joint_reply_data_request_port.set();
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		sr_ecp_msg.message("ECP get_reply epos_external_reply_data_request_port");
		// generator reply generation
		for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
			epos_external_reply_data_request_port.data.epos_controller[i].position =
					reply_package.smb.epos_controller[i].position;
			epos_external_reply_data_request_port.data.epos_controller[i].current =
					reply_package.smb.epos_controller[i].current;
			epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					reply_package.smb.epos_controller[i].motion_in_progress;
		}
		//	epos_external_reply_data_request_port.data.contact = reply_package.smb.contact;

		//	epos_external_reply_data_request_port.data.current_frame = reply_package.smb.current_pose;

		epos_external_reply_data_request_port.set();
	}

	if (smb_multi_leg_reply_data_request_port.is_new_request()) {
		smb_multi_leg_reply_data_request_port.data = reply_package.smb.multi_leg_reply;
		smb_multi_leg_reply_data_request_port.set();
	}

}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

