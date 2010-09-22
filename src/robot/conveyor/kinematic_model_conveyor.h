/*!
 * @file
 * @brief File containing the declaration of the class representing the conveyor kinematic model.
 *
 * @author tkornuta
 * @date 31.01.2007
 *
 * @ingroup KINEMATICS CONVEYOR_KINEMATICS conveyor
 */

#if !defined(_CONVEYOR_KIN_MODEL)
#define _CONVEYOR_KIN_MODEL

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace conveyor {

/*!
 *
 * @brief Class represents the conveyor kinematic model.
 *
 * @author tkornuta
 * @date Jan 04, 2010
 *
 * @ingroup KINEMATICS CONVEYOR_KINEMATICS
 */
class model : public common::kinematic_model
{
protected:
	//! Synchronization position.
	double synchro_motor_position;

	//! Motor increments to external position (in meters) ratio.
	double motor_to_intext_ratio;

	//! Method responsible for kinematic parameters setting.
	virtual void set_kinematic_parameters(void);

	/**
	 * @brief Checks whether given motor increments are valid.
	 * @param motor_position Motor position to be validated.
	 */
	virtual void check_motor_position(const lib::MotorArray & motor_position);

	/**
	 * @brief Checks whether given internal coordinates are valid.
	 * @param q Joints to be validated.
	 */
	virtual void check_joints(const lib::JointArray & q);

public:
	//! Constructor.
	model(void);

	/**
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	virtual void
		mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	/**
	 * @brief Computes motor increments from internal coordinates.
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	virtual void
		i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

};

} // namespace conveyor
} // namespace kinematic
} // namespace mrrocpp

#endif
