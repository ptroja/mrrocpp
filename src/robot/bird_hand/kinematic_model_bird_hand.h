/**
 * @file
 * @brief File containing declaration of the kinematic_model_bird_hand class.
 *
 * @author tkornuta
 * @author kczajkowski
 * @date May 28, 2010
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS bird_hand
 */

#ifndef KINEMATIC_MODEL_BIRD_HAND_H_
#define KINEMATIC_MODEL_BIRD_HAND_H_

#include "base/kinematics/kinematic_model.h"
#include "robot/bird_hand/kinematic_parameters_bird_hand.h"

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

/*!
 *
 * @brief Class solving the problem of inverse kinematics for Exechon parallel kinematc machine with spherical wrist attached to its upper platform.
 *
 * @author kczajkowski
 * @date May 28, 2010
 */
class kinematic_model_bird_hand : public common::kinematic_model
{
protected:
	//! Kinematic parameters of Bird Hand.
	kinematic_parameters_bird_hand params;

	//! Sets parameters used by given kinematics model - empty.
	void set_kinematic_parameters(void)
	{
	}

	/**
	 * @brief Checks whether given motor increments are valid.
	 *
	 * 	Empty due to the fact that the validation is implemented on the electronic circuit board mounted on bird-hand.
	 *
	 * @param motor_position Motor position to be validated.
	 */
	void check_motor_position(const lib::MotorArray & motor_position)
	{
	}

	/**
	 * @brief Checks whether given internal coordinates are valid.
	 * @param q Joints to be validated.
	 */
	void check_joints(const lib::JointArray & q);

public:
	//! Constructor.
	kinematic_model_bird_hand(void);

	/**
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	virtual void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	/**
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 *
	 * Values of the synchronization positions are substracted from the result.
	 *
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	void mp2i_transform_synch(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	/**
	 * @brief Computes motor increments from internal coordinates (i2mp from internal to motor position).
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	virtual void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

	/**
	 * @brief Computes motor increments from internal coordinates (i2mp from internal to motor position).
	 *
	 * In the end the result is summed with (the synchronization positions multiplicated by gear).
	 *
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	void i2mp_transform_synch(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

	/**
	 * @brief Solves direct kinematics - not (and will not be) implemented.
	 * @param[in] local_current_joints Given internal (joints) values.
	 * @param[out] local_current_end_effector_frame Computed end-effector frame (a homogeneous matrix).
	 */
	void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
	{
	}

	/**
	 * @brief Solves inverse kinematics - not (and will not be) implemented.
	 * @param[out] local_desired_joints Computed join values.
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	void inverse_kinematics_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
	{
	}

};

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp


#endif

