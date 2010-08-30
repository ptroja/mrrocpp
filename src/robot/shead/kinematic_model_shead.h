/*!
 * @file
 * @brief File containing the declaration of the kinematic model for the SwarmItFix agent's head class.
 *
 * @author tkornuta
 * @date 2010.02.01
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
 * @author yoyek
 * @date 2010.02.01
 *
 * @ingroup KINEMATICS SIF_KINEMATICS
 */
class model : public common::kinematic_model
{
protected:
	//! Method responsible for kinematic parameters setting.
	void set_kinematic_parameters(void);

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
	model(void);

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

} // namespace smb
} // namespace kinematic
} // namespace mrrocpp

#endif
