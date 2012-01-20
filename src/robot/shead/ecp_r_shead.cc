/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "base/lib/impconst.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/shead/ecp_r_shead.h"
#include "robot/shead/kinematic_model_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

robot::robot(const lib::robot_name_t & _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::_ecp_robot <lib::shead::c_buffer, lib::shead::r_buffer>(_robot_name, lib::shead::NUM_OF_SERVOS, _config, _sr_ecp) ,
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager),
		epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager),
		epos_brake_command_data_port(lib::epos::EPOS_QUICKSTOP_COMMAND_DATA_PORT, port_manager),
		epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager),

		solidification_data_port(lib::shead::SOLIDIFICATION_ACTIVATION_DATA_PORT, port_manager),
		vacuum_activation_data_port(lib::shead::VACUUM_ACTIVATION_DATA_PORT, port_manager),

		epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager),
		epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager),

		shead_reply_data_request_port(lib::shead::REPLY_DATA_REQUEST_PORT, port_manager)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
	data_ports_used = true;
}

robot::robot(const lib::robot_name_t & _robot_name, common::task::task_base& _ecp_object) :
		ecp::common::robot::_ecp_robot <lib::shead::c_buffer, lib::shead::r_buffer>(_robot_name, lib::shead::NUM_OF_SERVOS, _ecp_object) ,
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager),
		epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager),
		epos_brake_command_data_port(lib::epos::EPOS_QUICKSTOP_COMMAND_DATA_PORT, port_manager),
		epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager),

		solidification_data_port(lib::shead::SOLIDIFICATION_ACTIVATION_DATA_PORT, port_manager),
		vacuum_activation_data_port(lib::shead::VACUUM_ACTIVATION_DATA_PORT, port_manager),

		epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager),
		epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager),

		shead_reply_data_request_port(lib::shead::REPLY_DATA_REQUEST_PORT, port_manager)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
	data_ports_used = true;
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::shead::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

void robot::create_command()
{
	if (epos_motor_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}

		ecp_command.shead.variant = lib::shead::POSE;

		ecp_command.shead.set_pose_specification = lib::shead::MOTOR;

		for (int i = 0; i < lib::shead::NUM_OF_SERVOS; ++i) {
			ecp_command.shead.motor_pos[i] = epos_motor_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_joint_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.shead.variant = lib::shead::POSE;

		ecp_command.shead.set_pose_specification = lib::shead::JOINT;

		for (int i = 0; i < lib::shead::NUM_OF_SERVOS; ++i) {
			ecp_command.shead.joint_pos[i] = epos_joint_command_data_port.data.desired_position[i];
		}

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

		ecp_command.shead.variant = lib::shead::QUICKSTOP;

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
		ecp_command.shead.variant = lib::shead::CLEAR_FAULT;

		check_then_set_command_flag(is_new_data);
	}

	if (solidification_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		// generator command interpretation
		// narazie proste przepisanie

		ecp_command.shead.variant = lib::shead::SOLIDIFICATION;

		ecp_command.shead.head_solidification = solidification_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (vacuum_activation_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		// generator command interpretation
		// narazie proste przepisanie

		ecp_command.shead.variant = lib::shead::VACUUM;

		ecp_command.shead.vacuum_activation = vacuum_activation_data_port.data;

		check_then_set_command_flag(is_new_data);

	}

	if (epos_motor_reply_data_request_port.is_new_request()) {
		ecp_command.shead.get_pose_specification = lib::shead::MOTOR;
		//	ecp_command.get_arm_type = lib::MOTOR;
		//sr_ecp_msg.message("epos_motor_reply_data_request_port");

		check_then_set_command_flag(is_new_request);
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		ecp_command.shead.get_pose_specification = lib::shead::JOINT;
		//ecp_command.get_arm_type = lib::JOINT;
		//	sr_ecp_msg.message("epos_joint_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	is_new_request = is_new_request || shead_reply_data_request_port.is_new_request();

}

void robot::get_reply()
{

	// generator reply generation

	if (epos_motor_reply_data_request_port.is_new_request()) {
		// generator reply generation
		epos_motor_reply_data_request_port.data.epos_controller[0].position =
				reply_package.shead.epos_controller.position;
		epos_motor_reply_data_request_port.data.epos_controller[0].current =
				reply_package.shead.epos_controller.current;
		epos_motor_reply_data_request_port.data.epos_controller[0].motion_in_progress =
				reply_package.shead.epos_controller.motion_in_progress;
		epos_motor_reply_data_request_port.set();
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		// generator reply generation
		sr_ecp_msg.message("ECP get_reply epos_joint_reply_data_request_port");

		epos_joint_reply_data_request_port.data.epos_controller[0].position =
				reply_package.shead.epos_controller.position;
		epos_joint_reply_data_request_port.data.epos_controller[0].current =
				reply_package.shead.epos_controller.current;
		epos_joint_reply_data_request_port.data.epos_controller[0].motion_in_progress =
				reply_package.shead.epos_controller.motion_in_progress;
		//	epos_joint_reply_data_request_port.data.contact = reply_package.shead.contact;

		epos_joint_reply_data_request_port.set();
	}

	if (shead_reply_data_request_port.is_new_request()) {
		shead_reply_data_request_port.data = reply_package.shead.shead_reply;

		shead_reply_data_request_port.set();
	}

}

} // namespace shead
} // namespace ecp
} // namespace mrrocpp

