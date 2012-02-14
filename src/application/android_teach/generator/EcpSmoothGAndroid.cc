/*
 * EcpGAndroid.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#include <cmath>

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp/ecp_robot.h"
#include "application/android_teach/generator/EcpSmoothGAndroid.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

EcpSmoothGAndroid::EcpSmoothGAndroid(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose, int jointNumber, ecp_mp::sensor::EcpMpAndroid* androidSensor, ecp_mp::sensor::android_teach::AndroidState* androidState) :
		common::generator::newsmooth(_ecp_task, pose, jointNumber),
		androidSensor(androidSensor),
		androidState(androidState)
{
//    for(int i  = 0;i < MAX_NO_OF_DEGREES;++i)
//    {
//        currentValue[i] = 0;
//        requestedChange[i] = 0;
//        nextChange[i] = 0;
//        maxChange[i] = 0;
//        multipliers[i] = 0;
//    }
//
//    currentGripperValue = 0;

}

EcpSmoothGAndroid::~EcpSmoothGAndroid()
{
	// TODO Auto-generated destructor stub
}

bool EcpSmoothGAndroid::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.get_arm_type = lib::JOINT;

	androidState->updateCurrentJointPosition(the_robot->reply_package.arm.pf_def.joint_coordinates);

	bool result = common::generator::newsmooth::next_step();
	if (result) {
//		perror("result from newsmooth is true");
		result = !androidSensor->isAbort();
//		if (result)
//		{
//			perror("result from abort is true");
//
//		}
//		else
//			perror("result from abort is false");

	} else {
//		perror("result from newsmooth is false");
//
//		perror("smooth next_step send end of motion");
		//androidSensor->sendEndOfMotion();
	}

	if (androidState->disconnected)
		return false;

	return result;
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

