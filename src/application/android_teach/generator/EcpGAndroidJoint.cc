/*
 * EcpGAndroidJoint.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#include "EcpGAndroidJoint.h"

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "application/android_teach/generator/EcpGAndroidJoint.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

EcpGAndroidJoint::EcpGAndroidJoint(common::task::task& _ecp_task, ecp_mp::sensor::EcpMpAndroid* androidSensor, ecp_mp::sensor::android_teach::AndroidState* androidState) :
		EcpGAndroid(_ecp_task, androidSensor, androidState)
{
	int i;
	char buffer[100];

	for (i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		sprintf(buffer, "joint_multiplier_%d", i);
		multipliers[i] = ecp_t.config.value <double>(buffer);
		sprintf(buffer, "joint_max_change_%d", i);
		maxChange[i] = ecp_t.config.value <double>(buffer);

	}

}

EcpGAndroidJoint::~EcpGAndroidJoint()
{
	// TODO Auto-generated destructor stub
}

void EcpGAndroidJoint::set_position(bool changed)
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	//bylo for(int i = 0; i <= 7; ++i)
	for (int i = 0; i <= NUMBER_OF_JOINTS; ++i) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = nextChange[i];
	}
}

bool EcpGAndroidJoint::first_step()
{
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//    the_robot->ecp_command.get_arm_type = lib::JOINT;  // było FRAME
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::JOINT;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 25;
	the_robot->ecp_command.value_in_step_no = 22;

	releasedA = false;
	stop = false;

	return true;
}

void EcpGAndroidJoint::preset_position(void)
{
	int i;
	for (i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		requestedChange[i] = 0;
	}
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
