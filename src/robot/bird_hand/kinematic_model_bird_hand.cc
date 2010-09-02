/*!
 * @file
 * @brief File containing definition of kinematic_model_bird_hand class methods.
 *
 * @author kczajkowski
 * @date May 28, 2010
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS bird_hand
 */

#include <cmath>

#include "base/lib/com_buf.h"
#include "robot/bird_hand/kinematic_model_bird_hand.h"

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

kinematic_model_bird_hand::kinematic_model_bird_hand(void)
{
	// Set model name.
	set_kinematic_model_label("BIRD_HAND kinematic model");
}

void kinematic_model_bird_hand::check_joints(const lib::JointArray & q)
{

	if (isnan(q[0]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_D0);
	if (q[0] < params.lower_limit_joint[0])
		throw NonFatal_error_2(BEYOND_LOWER_D0_LIMIT);
	else if (q[0] > params.upper_limit_joint[0])
		throw NonFatal_error_2(BEYOND_UPPER_D0_LIMIT);

	if (isnan(q[1]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA1);
	if (q[1] < params.lower_limit_joint[1])
		throw NonFatal_error_2(BEYOND_LOWER_THETA1_LIMIT);
	else if (q[1] > params.upper_limit_joint[1])
		throw NonFatal_error_2(BEYOND_UPPER_THETA1_LIMIT);

	if (isnan(q[2]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA2);
	if (q[2] < params.lower_limit_joint[2])
		throw NonFatal_error_2(BEYOND_LOWER_THETA2_LIMIT);
	else if (q[2] > params.upper_limit_joint[2])
		throw NonFatal_error_2(BEYOND_UPPER_THETA2_LIMIT);

	if (isnan(q[3]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA3);
	if (q[3] < params.lower_limit_joint[3])
		throw NonFatal_error_2(BEYOND_LOWER_THETA3_LIMIT);
	else if (q[3] > params.upper_limit_joint[3])
		throw NonFatal_error_2(BEYOND_UPPER_THETA3_LIMIT);

	if (isnan(q[4]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA4);
	if (q[4] < params.lower_limit_joint[4])
		throw NonFatal_error_2(BEYOND_LOWER_THETA4_LIMIT);
	else if (q[4] > params.upper_limit_joint[4])
		throw NonFatal_error_2(BEYOND_UPPER_THETA4_LIMIT);

	if (isnan(q[5]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA5);
	if (q[5] < params.lower_limit_joint[5])
		throw NonFatal_error_2(BEYOND_LOWER_THETA5_LIMIT);
	else if (q[5] > params.upper_limit_joint[5])
		throw NonFatal_error_2(BEYOND_UPPER_THETA5_LIMIT);

	//	if (isnan(q[6]))
	//		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA6);
	//	if (q[6] < params.lower_limit_joint[6])
	//		throw NonFatal_error_2(BEYOND_LOWER_THETA6_LIMIT);
	//	else if (q[6] > params.upper_limit_joint[6])
	//		throw NonFatal_error_2(BEYOND_UPPER_THETA6_LIMIT);
	//
	//	if (isnan(q[7]))
	//		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA7);
	//	if (q[7] < params.lower_limit_joint[7])
	//		throw NonFatal_error_2(BEYOND_LOWER_THETA7_LIMIT);
	//	else if (q[7] > params.upper_limit_joint[7])
	//		throw NonFatal_error_2(BEYOND_UPPER_THETA7_LIMIT);
}

void kinematic_model_bird_hand::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{

	for (int i = 0; i < 8; ++i)
		local_desired_motor_pos_new[i] = local_desired_joints[i] * params.gear[i];

}

void kinematic_model_bird_hand::i2mp_transform_synch(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{

	//    for (int i=0; i<8; ++i){
	//          printf("[info] local_desired_joints[%d]: %f \n", i, local_desired_joints[i]);
	//          fflush(stdout);
	//    }
	check_joints(local_desired_joints);

	for (int i = 0; i < 8; ++i)
		local_desired_motor_pos_new[i] = (local_desired_joints[i] + params.synchro_joint_position[i]) * params.gear[i];

}

void kinematic_model_bird_hand::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{

	for (int i = 0; i < 8; ++i)
		local_current_joints[i] = local_current_motor_pos[i] / params.gear[i];

}

void kinematic_model_bird_hand::mp2i_transform_synch(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{

	for (int i = 0; i < 8; ++i)
		local_current_joints[i] = local_current_motor_pos[i] / params.gear[i] - params.synchro_joint_position[i];

	//    for (int i=0; i<8; ++i){
	//          printf("[info] local_current_joints[%d]: %f \n", i, local_current_joints[i]);
	//          fflush(stdout);
	//    }
	//check_joints(local_current_joints);

}

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp

