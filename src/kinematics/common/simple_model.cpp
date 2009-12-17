/*!
 * \file simple_model.cpp
 * \brief File containing definitions of simple_model class methods.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

#include "simple_model.h"

namespace mrrocpp {
namespace kinematic {
namespace common {

simple_model::simple_model()
{

}

simple_model::~simple_model()
{

}

const char* simple_model::get_kinematic_model_label(void)
{
	return kinematic_model_label.c_str();
}

void simple_model::set_kinematic_model_label(const char * _label)
{
	kinematic_model_label = std::string(_label);
}


void simple_model::i2e_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame)
{
	// Direct kinematics solution.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);
}


void simple_model::e2i_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame)
{
	// Inverse kinematics solution.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_frame);

}


} // namespace common
} // namespace kinematic
} // namespace mrrocpp

