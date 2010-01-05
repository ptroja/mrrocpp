/*!
 * \file kinematic_model_spkm.cc
 * \brief File containing definition of kinematic_model_spkm class methods.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */

#include <math.h>

#include "lib/com_buf.h"
#include "kinematics/spkm/kinematic_model_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

kinematic_model_spkm::kinematic_model_spkm(void)
{
	// Set model name.
	set_kinematic_model_label("SPKM main kinematic model");
	// Set kinematic model parameters.
	set_kinematic_parameters();
}

void kinematic_model_spkm::set_kinematic_parameters(void)
{
}

void kinematic_model_spkm::check_motor_position(const double motor_position[])
{
}

void kinematic_model_spkm::check_joints(const double q[])
{
}

void kinematic_model_spkm::mp2i_transform(const double* local_current_motor_pos, double* local_current_joints)
{
}

void kinematic_model_spkm::i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints)
{
}

void kinematic_model_spkm::direct_kinematics_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame)
{
}

void kinematic_model_spkm::inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame)
{
}

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp

