/*!
 * \file kinematic_model.cpp
 * \brief File containing definitions of kinematic_model class methods.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

#include "kinematics/common/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

const char* kinematic_model::get_kinematic_model_label(void)
{
	return label.c_str();
}

void kinematic_model::set_kinematic_model_label(const std::string  _label)
{
	label = _label;
}


void kinematic_model::i2e_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame)
{
	// Direct kinematics solution.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);
}


void kinematic_model::e2i_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame)
{
	// Inverse kinematics solution.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_frame);

}


} // namespace common
} // namespace kinematic
} // namespace mrrocpp

