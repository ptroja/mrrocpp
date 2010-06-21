/*
 * \file kinematic_model_bird_hand.h
 * \brief File containing declaration of the kinematic_model_bird_hand class.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */

#ifndef KINEMATIC_MODEL_BIRD_HAND_H_
#define KINEMATIC_MODEL_BIRD_HAND_H_

#include "kinematics/common/kinematic_model.h"
#include "kinematics/bird_hand/kinematic_parameters_bird_hand.h"

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

/*!
 * \class kinematic_model_bird_hand
 * \brief Class solving the problem of inverse kinematics for Exechon parallel kinematc machine with spherical wrist attached to its upper platform.
 *
 * \author kczajkowski
 * \date May 28, 2010
 */
class kinematic_model_bird_hand: public common::kinematic_model {
protected:
	//! Kinematic parameters of Bird Hand
	kinematic_parameters_bird_hand params;

	//! Sets parameters used by given kinematics model - implemented in kinematic_parameters_bird_hand constructor
	void set_kinematic_parameters(void) {
	}

	//! Checks whether given motor increments are valid.
	void check_motor_position(const lib::MotorArray & motor_position);

	//! Checks whether given internal coordinates are valid.
	void check_joints(const lib::JointArray & q);

public:
	//! Constructor.
	kinematic_model_bird_hand(void);

	//! Computes joint values basing on the motor increments.
	void mp2i_transform(const lib::MotorArray & local_current_motor_pos,
			lib::JointArray & local_current_joints);

	//! Computes motor increments from joint values (i2mp from internal to motor position).
	void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new,
			lib::JointArray & local_desired_joints);

	//! Solves direct kinematics - will not be implemented
	void direct_kinematics_transform(
			const lib::JointArray & local_current_joints,
			lib::Homog_matrix& local_current_end_effector_frame) {
	}

	//! Solves inverse kinematics - will not be implemented
	void inverse_kinematics_transform(lib::JointArray & local_desired_joints,
			lib::JointArray & local_current_joints,
			const lib::Homog_matrix& local_desired_end_effector_frame) {
	}

};

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp


#endif

