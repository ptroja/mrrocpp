/*!
 * \file kinematic_model.h
 * \brief File containing the declaration of kinematic_model class.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

#ifndef KINEMATIC_MODEL_H_
#define KINEMATIC_MODEL_H_

#include <string>
#include <vector>
#include "lib/mrmath/mrmath.h"

#include "lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 * \class kinematic_model
 * \brief Base and simplest class of all kinematic models. Its simplicity is related to the fact, that it offers only
 * basic six kinematic methods: direct, inverse, i2e, e2i, mp2i and i2mp.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */
class kinematic_model {
protected:
	//! Name of given kinematics.
	std::string label;

	//! Sets parameters used by given kinematics.
	virtual void set_kinematic_parameters(void) = 0;

	//! Checks whether given motor increments are valid.
	virtual void check_motor_position(const lib::MotorArray & motor_position) = 0;

	//! Checks whether given internal coordinates are valid.
	virtual void check_joints(const lib::JointArray & q) = 0;

public:

	//! Class virtual destructor - empty.
	virtual ~kinematic_model() { }

	//! Computes internal coordinates basing on the motor increments (position).
	virtual void mp2i_transform(const lib::MotorArray &local_current_motor_pos, lib::JointArray & local_current_joints) = 0;

	//! Computes motor increments from internal coordinates.
	virtual void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, lib::JointArray & local_desired_joints) = 0;

	//! Computes external coordinates on the base of internal coordinates (i2e - internal to external). In this kinematic_model_with_tool calls only one method - direct kinematics.
	virtual void i2e_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	//! Computes internal coordinates basing on external coordinates (e2i - external to internal). Calls only one method - inverse kinematics.
	virtual void e2i_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

	//! Solves direct kinematics.
	virtual void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	//! Solves inverse kinematics.
	virtual void inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix & local_desired_end_effector_frame);

	//! Sets kinematics description.
	virtual void set_kinematic_model_label(const std::string &);

	//! Returns description of kinematics.
	virtual std::string get_kinematic_model_label(void);
};

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_MODEL_H_ */
