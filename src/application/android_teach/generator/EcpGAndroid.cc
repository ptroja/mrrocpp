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
#include "application/android_teach/generator/EcpGAndroid.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

EcpGAndroid::EcpGAndroid(common::task::task& _ecp_task, ecp_mp::sensor::EcpMpAndroid* androidSensor, ecp_mp::sensor::android_teach::AndroidState* androidState) :
		common::generator::generator(_ecp_task), androidSensor(androidSensor), androidState(androidState)
{
	for (int i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		currentValue[i] = 0;
		requestedChange[i] = 0;
		nextChange[i] = 0;
		maxChange[i] = 0;
		multipliers[i] = 0;
	}

	currentGripperValue = 0;

}

EcpGAndroid::~EcpGAndroid()
{
	// TODO Auto-generated destructor stub
}

bool EcpGAndroid::calculate_change(int axis, double value)
{
	bool changed = false;

	value *= multipliers[axis];
	requestedChange[axis] = value;

	for (int i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		if (fabs(nextChange[i] - requestedChange[i]) < maxChange[i]) {
			nextChange[i] = requestedChange[i];
		} else {
			if (requestedChange[i] > nextChange[i])
				nextChange[i] = nextChange[i] + maxChange[i];
			else
				nextChange[i] = nextChange[i] - maxChange[i];
		}
		if (nextChange[i] != 0)
			changed = true;
	}

	return changed;
}

int EcpGAndroid::get_axis(void)
{
	int axis = -1;
	axis = androidState->getSelectedAxis();
	return axis;
}

bool EcpGAndroid::next_step()
{
	char buf[100];
	androidState->updateCurrentJointPosition(the_robot->reply_package.arm.pf_def.joint_coordinates);

	androidSensor->getReadings();

	if (androidState->readings.mode == 2) {
		perror("	next_step() mode = 2  zle");
	}

	if (androidState->readings.value == 0) {
		perror("stop w next_step EcpGAndroid");
	}

//    for(int i=0; i < NUMBER_OF_JOINTS; ++i)
//    {
//    	androidState->ecpStatus.jointValue[i] = (double)the_robot->reply_package.current[i+1]/100.0;
//		the_robot->create_command();
//		the_robot->get_reply();
//		sprintf(buf,"robot 2 value: %f",the_robot->reply_package.arm.pf_def.arm_coordinates[2]);
//    	perror(buf);
//    }

	++step_no;

	int speed = androidState->readings.value;

	//By OL
	if (speed == 0)
		stop = true;

	int axis = get_axis();

	double value = speed * androidState->jointSpeedMulti[axis];

	preset_position();

	// bylo for(int i = 0; i < 7; ++i)
	for (int i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		requestedChange[i] = 0;
	}

	bool changed = calculate_change(axis, value);

//	if (androidState->readings.value != 0)
//	{
//		sprintf(buf,"axis: %d ; speed: %d ; value: %f ; currentpos: %f ; change: %f",axis, speed, value, androidState->ecpStatus.jointValue[axis], the_robot->ecp_command.arm.pf_def.arm_coordinates[axis]);
//		perror(buf);
//	}

	if (!changed && stop)
		return false;

	if (androidState->disconnected)
		return false;

	set_position(changed);

	return true;

}

void EcpGAndroid::execute_motion(void)
{
	// komunikacja wlasciwa z manipulatorem
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {
		the_robot->query();
		//throw common::robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
		throw ecp_mp::sensor::android_teach::NetworkException();
	}
	the_robot->query();

	if (the_robot->reply_package.reply_type == lib::ERROR) {
		switch (the_robot->reply_package.error_no.error0)
		{
			case BEYOND_UPPER_D0_LIMIT:
			case BEYOND_UPPER_THETA1_LIMIT:
			case BEYOND_UPPER_THETA2_LIMIT:
			case BEYOND_UPPER_THETA3_LIMIT:
			case BEYOND_UPPER_THETA4_LIMIT:
			case BEYOND_UPPER_THETA5_LIMIT:
			case BEYOND_UPPER_THETA6_LIMIT:
			case BEYOND_UPPER_THETA7_LIMIT:
			case BEYOND_UPPER_LIMIT_AXIS_1:
			case BEYOND_UPPER_LIMIT_AXIS_2:
			case BEYOND_UPPER_LIMIT_AXIS_3:
			case BEYOND_UPPER_LIMIT_AXIS_4:
			case BEYOND_UPPER_LIMIT_AXIS_5:
			case BEYOND_UPPER_LIMIT_AXIS_6:
			case BEYOND_UPPER_LIMIT_AXIS_7:
				androidState->setLimit(1);
				break;
			case BEYOND_LOWER_D0_LIMIT:
			case BEYOND_LOWER_THETA1_LIMIT:
			case BEYOND_LOWER_THETA2_LIMIT:
			case BEYOND_LOWER_THETA3_LIMIT:
			case BEYOND_LOWER_THETA4_LIMIT:
			case BEYOND_LOWER_THETA5_LIMIT:
			case BEYOND_LOWER_THETA6_LIMIT:
			case BEYOND_LOWER_THETA7_LIMIT:
			case BEYOND_LOWER_LIMIT_AXIS_1:
			case BEYOND_LOWER_LIMIT_AXIS_2:
			case BEYOND_LOWER_LIMIT_AXIS_3:
			case BEYOND_LOWER_LIMIT_AXIS_4:
			case BEYOND_LOWER_LIMIT_AXIS_5:
			case BEYOND_LOWER_LIMIT_AXIS_6:
			case BEYOND_LOWER_LIMIT_AXIS_7:
				androidState->setLimit(2);
				break;
			default:
				throw ecp_mp::sensor::android_teach::NetworkException(); //common::robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
				break;

		} /* end: switch */
	} else {
		androidState->setLimit(0);
	}
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

