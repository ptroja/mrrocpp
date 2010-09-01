/*!
 * @file
 * @brief File containing base kinematic model for two fingered grippers.
 *
 * @author yoyek
 * @author tkornuta
 * @date Jun 21, 2010
 *
 * @ingroup KINEMATICS irp6_tfg
 */

#if !defined(_IRP6_TFG_KIN_MODEL)
#define _IRP6_TFG_KIN_MODEL

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6_tfg {

/*!
 *
 * @brief Base kinematic model for two fingered grippers.
 *
 * @author tkornuta
 * @date Jun 21, 2010
 *
 * @ingroup KINEMATICS
 */
class kinematic_model_irp6_tfg : public common::kinematic_model
{
protected:

	//! Lower limit of motor movement.
	double lower_limit_axis;

	//! Upper limit of motor movement.
	double upper_limit_axis;

	//! Lower limit of joint movement (in radians).
	double lower_limit_joint;

	//! Upper limit of joint movement (in radians).
	double upper_limit_joint;

	//! Motor synchronization position.
	double synchro_motor_position;

	//! Internal (joint) synchronization position.
	double synchro_joint_position;

	//! Variable related to the computations of the gripper spread.
	double dir_a_7;

	//! Variable related to the computations of the gripper spread.
	double dir_b_7;

	//! Variable related to the computations of the gripper spread.
	double dir_c_7;

	//! Variable related to the computations of the gripper spread.
	double inv_a_7;

	//! Variable related to the computations of the gripper spread.
	double inv_b_7;

	//! Variable related to the computations of the gripper spread.
	double inv_c_7;

	//! Variable related to the computations of the gripper spread.
	double inv_d_7;

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
	//! Constructor.
	kinematic_model_irp6_tfg(void);

	/**
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	/**
	 * @brief Computes motor increments from internal coordinates.
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

};

} // namespace irp6_tfg
} // namespace kinematic
} // namespace mrrocpp

#endif
