/*!
 * @file
 * @brief File containing the IRp-6p with wrist (6DOFs) kinematic model class.
 *
 * @author tkornuta
 * @date 31.01.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS irp6p_m
 */


#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST)
#define _IRP6P_KIN_MODEL_WITH_WRIST

#include "base/kinematics/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

/*!
 *
 * @brief The kinematic model utilizing six IRP-6p DOFs.
 *
 * The model_with_wrist kinematic model utilizes six (with an additional one in the wrist) IRP-6p DOFs.
 *
 * @author tkornuta
 * @date 31.01.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS
 */
class model_with_wrist: public common::kinematic_model_with_tool
{
protected:
	//! D-H kinematic parameters - length of 2nd segment.
	double a2;

	//! D-H kinematic parameters - length of 3rd segment.
	double a3;

	//! D-H kinematic parameters - length of 4th segment.
	double d5;

	//! D-H kinematic parameters - length of 5th segment.
	double d6;

	//! D-H kinematic parameters - length of 6th segment.
	double d7;

	//! Table storing gear ratio for all DOFs.
	double gear[7];

	//! Variable storing gear additional rotation for all DOFs.
	double theta[7];

	//! Variable utilized in computations related to upper and lower arm.
	double sl123;
	//! Variable utilized in computations related to upper and lower arm.
	double mi1;
	//! Variable utilized in computations related to upper and lower arm.
	double ni1;
	//! Variable utilized in computations related to upper and lower arm.
	double mi2;
	//! Variable utilized in computations related to upper and lower arm.
	double ni2;

	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double dir_a_6;
	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double dir_b_6;
	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double dir_c_6;
	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double inv_a_6;
	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double inv_b_6;
	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double inv_c_6;
	//! DEPRICATED: Variable related to the computations of the gripper spread (moved to gripper kinematics).
	double inv_d_6;

	//! Lower limits of motor movement.
	double lower_limit_axis[7];

	//! Upper limits of motor movement.
	double upper_limit_axis[7];

	//! Lower limit of joint movement (in radians).
	double lower_limit_joint[7];

	//! Upper limit of joint movement (in radians).
	double upper_limit_joint[7];


	//! Synchronization positions of each motor - in motor increments.
	double synchro_motor_position[7];

	//! Synchronization positions of each joint - in internal coordinates.
	double synchro_joint_position[7];

	//! Method responsible for kinematic parameters setting.
	virtual void set_kinematic_parameters(void);

	/**
	 * @brief Checks whether given motor increments are valid.
	 * @param motor_position Motor position to be validated.
	 */
	void check_motor_position(const lib::MotorArray & motor_position);

	/**
	 * @brief Checks whether given internal coordinates are valid.
	 * @param q Joints to be validated.
	 */
	void check_joints(const lib::JointArray & q);

public:
	//! Number of degrees of freedom (thus joints and servos).
	int number_of_servos;

	/**
	 * @brief Constructor.
	 * @param _number_of_servos Number of servos (joints).
	 */
	model_with_wrist(int _number_of_servos);

	/**
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	virtual void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	/**
	 * @brief Computes motor increments from internal coordinates.
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	virtual void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

	/**
	 * @brief Solves direct kinematics. The new, 6th DOF is active.
	 * @param[in] local_current_joints Given internal (joints) values (q0, q1, ...).
	 * @param[out] local_current_end_effector_frame Computed end-effector frame (a homogeneous matrix).
	 */
	virtual void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	/**
	 * @brief Solves inverse kinematics. The new, 6th DOF is active.
	 * @param[out] local_desired_joints Computed join values (q0, q1, ...).
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

};

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

#endif
