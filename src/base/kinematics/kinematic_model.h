/*!
 * @file
 * @brief File containing the declaration of the kinematic_model class.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */

#ifndef KINEMATIC_MODEL_H_
#define KINEMATIC_MODEL_H_

#include <string>
#include <vector>
#include "base/lib/mrmath/mrmath.h"

#include "base/lib/exception.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 *
 * @brief Base and simplest class of all kinematic models.
 *
 * Class simplicity is related to the fact, that it offers only
 * basic six kinematic methods: direct, inverse, i2e, e2i, mp2i and i2mp.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */
class kinematic_model
{
protected:
	//! Name of given kinematics.
	std::string label;

	//! Sets parameters used by given kinematics.
	virtual void set_kinematic_parameters(void) = 0;

	/**
	 * @brief Checks whether given motor increments are valid.
	 * @param motor_position Motor position to be validated.
	 */
	virtual void check_motor_position(const lib::MotorArray & motor_position) = 0;

	/**
	 * @brief Checks whether given internal coordinates are valid.
	 * @param q Joints to be validated.
	 */
	virtual void check_joints(const lib::JointArray & q) = 0;

public:

	//! Class virtual destructor - empty.
	virtual ~kinematic_model()
	{
	}

	/**
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	virtual void
			mp2i_transform(const lib::MotorArray &local_current_motor_pos, lib::JointArray & local_current_joints) = 0;

	/**
	 * @brief Computes motor increments from internal coordinates.
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	virtual void
			i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints) = 0;

	/**
	 * @brief Computes external coordinates on the base of internal coordinates (i2e - internal to external).
	 * In this (the simplest) kinematic model it calls only one method - direct kinematics.
	 * @param[in] local_current_joints Current joints values.
	 * @param[out] local_current_end_effector_frame Homogeneous matrix with computed end effector frame.
	 */
	virtual void
			i2e_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	/**
	 * @brief Computes internal coordinates basing on external coordinates (e2i - external to internal).
	 * Calls only one method - inverse kinematics.
	 * @param[out] local_desired_joints Computed join values.
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void
			e2i_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

	/**
	 * @brief Solves direct kinematics. Should be reimplemented by every robot/kinematic model.
	 * @param[in] local_current_joints Given internal (joints) values.
	 * @param[out] local_current_end_effector_frame Computed end-effector frame (a homogeneous matrix).
	 */
	virtual void
			direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	/**
	 * @brief Solves inverse kinematics. Should be reimplemented by every robot/kinematic model.
	 * @param[out] local_desired_joints Computed join values.
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void
			inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix & local_desired_end_effector_frame);

	/**
	 * @brief Sets kinematics description.
	 * @param _label Kinematics description to be set.
	 */
	virtual void set_kinematic_model_label(const std::string & _label);

	/**
	 * @brief Returns description of kinematics.
	 * @return Kinematics description (label).
	 */
	virtual std::string get_kinematic_model_label(void);
};

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_MODEL_H_ */
