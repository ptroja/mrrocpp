/*!
 * @file
 * @brief File containing methods of the kinematic model for the SwarmItFix agent's head class.
 *
 * @author tkornuta
 * @date 2010.02.01
 *
 * @ingroup KINEMATICS SIF_KINEMATICS sbench
 */

#include "base/lib/com_buf.h"
#include "robot/sbench/kinematic_model_sbench.h"

namespace mrrocpp {
namespace kinematics {
namespace sbench {

model::model(void)
{
	// Set label.
	set_kinematic_model_label("Switching to simple kinematic model");

	// Set kinematics parameters.
	set_kinematic_parameters();
}

void model::set_kinematic_parameters(void)
{
}

void model::check_motor_position(const lib::MotorArray & motor_position) const
{
}

void model::check_joints(const lib::JointArray & q) const
{
}

void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
}

void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
}

} // namespace smb
} // namespace kinematic
} // namespace mrrocpp

