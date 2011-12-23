/*!
 * @file
 * @brief File containing the declaration of the kinematic model for the SwarmItFix agent's head class.
 *
 * @ingroup KINEMATICS SIF_KINEMATICS shead
 */

#if !defined(_SHEAD_KIN_MODEL)
#define _SHEAD_KIN_MODEL

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace shead {

/*!
 *
 * @brief Kinematic model for the SwarmItFix agent's head class.
 *
 * @ingroup KINEMATICS SIF_KINEMATICS
 */
class model : public common::kinematic_model
{
protected:
	//! Method responsible for kinematic parameters setting.
	void set_kinematic_parameters(void);

	// Motor limits
	static lib::MotorArray::value_type lower_motor_limit, upper_motor_limit;

	// Joint limits
	static lib::JointArray::value_type lower_joint_limit, upper_joint_limit;

public:
	//! Constructor.
	model(void);

	/**
	 * @brief Checks whether given motor increments are valid.
	 * @param motor_position Motor position to be validated.
	 */
	void check_motor_position(const lib::MotorArray & motor_position) const;

	/**
	 * @brief Checks whether given internal coordinates are valid.
	 * @param q Joints to be validated.
	 */
	void check_joints(const lib::JointArray & q) const;

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

private:
	//! Parameter for conversion between joint and motor coordinates [rad]->[qc].
	static const double i2m_ratio;
};

} // namespace smb
} // namespace kinematic
} // namespace mrrocpp

#endif
