/*
 * ecp_g_constant_velocity.cpp
 *
 *  Created on: May 18, 2010
 *      Author: rtulwin
 */

#include "ecp_g_constant_velocity.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

constant_velocity::constant_velocity(common::task::task& _ecp_task, bool _is_synchronised, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
		multiple_position (_ecp_task) {
	this->pose_spec = pose_spec;
	this->axes_num = axes_num;
	this->vpc = velocity_profile_calculator::constant_velocity_profile();
}

constant_velocity::~constant_velocity() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity::first_step() {
	//TODO sprawdzic czy trajektoria jest wygenerowana
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	if (motion_type == lib::RELATIVE) {
		the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	} else {
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	}

	switch (pose_spec) {
		case lib::ECP_XYZ_ANGLE_AXIS || lib::ECP_XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
			the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
			if (motion_type == lib::RELATIVE) {
				the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
				for (int i=0; i<6; i++) {
					the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
				}
			} else {
				the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			}
			break;
		case lib::ECP_MOTOR:
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			break;
		case lib::ECP_JOINT:
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			break;
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}
}

bool constant_velocity::next_step() {

}

bool multiple_position::load_absolute_joint_trajectory_pose(vector<double> & coordinates) {
	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;
	vector<double> joint_max_velocity;
	for (int i = 0; i < axes_num; i++) {
		joint_max_velocity.push_back(0.05);
	}
	if (pose_vector.size() > 0 && pose_spec != lib::ECP_JOINT) {

		return false;
	}
	pose_spec = lib::ECP_JOINT;
	pose = ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose(lib::ECP_JOINT, coordinates, joint_max_velocity);

	pose_vector.push_back(pose);

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
