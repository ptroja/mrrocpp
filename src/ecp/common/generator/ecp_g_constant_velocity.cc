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
	// TODO Auto-generated constructor stub

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

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
