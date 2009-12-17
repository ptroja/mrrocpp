/*!
 * \file simple_model.h
 * \brief File containing the declaration of simple_model class.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

#ifndef SIMPLE_MODEL_H_
#define SIMPLE_MODEL_H_

#include <string>
#include "lib/mathtr/mrmath.h"

namespace mrrocpp {
namespace kinematic {
namespace common {

/*!
 * \class simple_model
 * \brief Base and simpless class of all kinematics models. Its simplicity is related to the fact, that it offers only
 * basic six kinematic methods: direct, inverse, i2e, e2i, mp2i and i2mp.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */
class simple_model {
protected:
	//! Name of given kinematics model.
	std::string kinematic_model_label;

	//! Sets parameters used by given kinematics.
	virtual void set_kinematic_parameters(void) = 0;

	//! Checks whether given motor increments are valid.
	virtual void check_motor_position(const double motor_position[]) = 0;

	//! Checks whether given internal coordinates are valid.
	virtual void check_joints(const double q[]) = 0;

public:
	//! Class constructor - empty.
	simple_model();

	//! Class virtual destructor - empty.
	virtual ~simple_model();

	//! Computes internal coordinates basing on the motor increments (position).
	virtual void mp2i_transform(const double* local_current_motor_pos, double* local_current_joints) = 0;

	//! Computes motor increments from internal coordinates.
	virtual void i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints) = 0;

	//! Computes external coordinates on the base of internal coordinates (i2e - internal to external). In this model calls only one method - direct kinematics.
	virtual void i2e_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame);

	//! Computes internal coordinates basing on external coordinates (e2i - external to internal). Calls only one method - inverse kinematics.
	virtual void e2i_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame);

	//! Solves direct kinematics.
	virtual void direct_kinematics_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame) = 0;

	//! Solves inverse kinematics.
	virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame) = 0;

	//! Sets kinematics model description.
	virtual void set_kinematic_model_label(const char*);

	//! Returns description of kinematics model.
	virtual const char* get_kinematic_model_label(void);

};

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif /* SIMPLE_MODEL_H_ */
