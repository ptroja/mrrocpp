/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/lib/impconst.h"

#include "ecp_r_spkm.h"
#include "base/lib/sr/sr_ecp.h"
#include "kinematic_model_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(const lib::robot_name_t & _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::_ecp_robot <lib::spkm::c_buffer, lib::spkm::r_buffer>(_robot_name, lib::spkm::NUM_OF_SERVOS, _config, _sr_ecp)
		,
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager)
		,
		epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager)
		,
		epos_external_command_data_port(lib::spkm::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager)
		,
		epos_quickstop_command_data_port(lib::epos::EPOS_QUICKSTOP_COMMAND_DATA_PORT, port_manager)
		,
		epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager)
		,
		epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager)
		,
		epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager)
		,
		epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager)
		,
		epos_external_reply_data_request_port(lib::spkm::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager)
{
	data_ports_used = true;
}

robot::robot(const lib::robot_name_t & _robot_name, common::task::task_base& _ecp_object) :
		ecp::common::robot::_ecp_robot <lib::spkm::c_buffer, lib::spkm::r_buffer>(_robot_name, lib::spkm::NUM_OF_SERVOS, _ecp_object)
		,
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager)
		,
		epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager)
		,
		epos_external_command_data_port(lib::spkm::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager)
		,
		epos_quickstop_command_data_port(lib::epos::EPOS_QUICKSTOP_COMMAND_DATA_PORT, port_manager)
		,
		epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager)
		,
		epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager)
		,
		epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager)
		,
		epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager)
		,
		epos_external_reply_data_request_port(lib::spkm::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager)
{
	data_ports_used = true;
}

void robot::create_command()
{

	// Set default variant to error in order to help tracking errors in communication
	// TODO: the following should be if-then-elseif-elseif-elseif...-else branch tree
	ecp_command.spkm.variant = (lib::spkm::CBUFFER_VARIANT) -1;

	if (epos_motor_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}

		ecp_command.spkm.variant = lib::spkm::POSE;

		ecp_command.spkm.set_pose_specification = lib::spkm::MOTOR;

		ecp_command.spkm.motion_variant = epos_motor_command_data_port.data.motion_variant;
		ecp_command.spkm.estimated_time = epos_motor_command_data_port.data.estimated_time;

		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; ++i) {
			ecp_command.spkm.motor_pos[i] = epos_motor_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_joint_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.spkm.variant = lib::spkm::POSE;

		ecp_command.spkm.set_pose_specification = lib::spkm::JOINT;

		ecp_command.spkm.motion_variant = epos_joint_command_data_port.data.motion_variant;
		ecp_command.spkm.estimated_time = epos_joint_command_data_port.data.estimated_time;
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; ++i) {
			ecp_command.spkm.joint_pos[i] = epos_joint_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_external_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.spkm.variant = lib::spkm::POSE;

		ecp_command.spkm.set_pose_specification = epos_external_command_data_port.data.pose_specification;

		ecp_command.spkm.motion_variant = epos_external_command_data_port.data.motion_variant;
		ecp_command.spkm.estimated_time = epos_external_command_data_port.data.estimated_time;

		for (int i = 0; i < 6; ++i) {
			ecp_command.spkm.goal_pos[i] = epos_external_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_quickstop_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}
		// generator command interpretation
		// narazie proste przepisanie

		ecp_command.spkm.variant = lib::spkm::QUICKSTOP;

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

		ecp_command.spkm.variant = lib::spkm::BRAKE;

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
		ecp_command.spkm.variant = lib::spkm::CLEAR_FAULT;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_motor_reply_data_request_port.is_new_request()) {
		ecp_command.spkm.get_pose_specification = lib::spkm::MOTOR;
		//	ecp_command.get_arm_type = lib::MOTOR;
		//sr_ecp_msg.message("epos_motor_reply_data_request_port");

		check_then_set_command_flag(is_new_request);
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		ecp_command.spkm.get_pose_specification = lib::spkm::JOINT;
		//ecp_command.get_arm_type = lib::JOINT;
		//	sr_ecp_msg.message("epos_joint_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	if (epos_external_reply_data_request_port.is_new_request()) {

		ecp_command.spkm.get_pose_specification = epos_external_reply_data_request_port.set_data;
		//ecp_command.get_arm_type = lib::FRAME;
		//sr_ecp_msg.message("epos_external_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

}

void robot::get_reply()
{

	if (epos_motor_reply_data_request_port.is_new_request()) {
		// generator reply generation
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_motor_reply_data_request_port.data.epos_controller[i].position =
					reply_package.spkm.epos_controller[i].position;
			epos_motor_reply_data_request_port.data.epos_controller[i].current =
					reply_package.spkm.epos_controller[i].current;
			epos_motor_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					reply_package.spkm.epos_controller[i].motion_in_progress;
		}
		epos_motor_reply_data_request_port.data.contact = reply_package.spkm.contact;

		epos_motor_reply_data_request_port.set();
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		// generator reply generation
		sr_ecp_msg.message("ECP get_reply epos_joint_reply_data_request_port");

		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_joint_reply_data_request_port.data.epos_controller[i].position =
					reply_package.spkm.epos_controller[i].position;
			epos_joint_reply_data_request_port.data.epos_controller[i].current =
					reply_package.spkm.epos_controller[i].current;
			epos_joint_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					reply_package.spkm.epos_controller[i].motion_in_progress;
		}
		epos_joint_reply_data_request_port.data.contact = reply_package.spkm.contact;

		epos_joint_reply_data_request_port.set();
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		sr_ecp_msg.message("ECP get_reply epos_external_reply_data_request_port");

		// generator reply generation
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_external_reply_data_request_port.data.epos_controller[i].position =
					reply_package.spkm.epos_controller[i].position;
			epos_external_reply_data_request_port.data.epos_controller[i].current =
					reply_package.spkm.epos_controller[i].current;
			epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					reply_package.spkm.epos_controller[i].motion_in_progress;
		}

		epos_external_reply_data_request_port.data.contact = reply_package.spkm.contact;

		for (int i = 0; i < 6; ++i) {
			epos_external_reply_data_request_port.data.current_pose[i] = reply_package.spkm.current_pose[i];
		}

		epos_external_reply_data_request_port.set();
	}

}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

