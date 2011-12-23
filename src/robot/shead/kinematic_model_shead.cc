/*!
 * @brief File containing methods of the kinematic model for the SwarmItFix agent's head class.
 *
 * @ingroup KINEMATICS SIF_KINEMATICS shead
 */

#include <cmath>

#include "base/lib/mrmath/MotorArray.h"
#include "base/lib/mrmath/JointArray.h"
#include "robot/shead/kinematic_model_shead.h"
#include "base/edp/edp_exceptions.h"

namespace mrrocpp {
namespace kinematics {
namespace shead {

/**
 * Encoder resolution: 2000*4 [qc/revolution].
 * Gear ratio: 100.
 */
const double model::i2m_ratio = 2000*4*100/(2*M_PI);

model::model(void)
{
	// Set label.
	set_kinematic_model_label("Switching to simple kinematic model");

	// Set kinematics parameters.
	set_kinematic_parameters();
}

void model::set_kinematic_parameters(void)
{
	// Empty.
}

// Motor limits definitions
lib::MotorArray::value_type model::lower_motor_limit = -235000;
lib::MotorArray::value_type model::upper_motor_limit = 5000;

void model::check_motor_position(const lib::MotorArray & motors) const
{
	if (motors[0] < lower_motor_limit) {
		BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(1) << limit_type(LOWER_LIMIT) << desired_value(motors[0]));
	}
	if (motors[0] > upper_motor_limit) {
		BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(1) << limit_type(UPPER_LIMIT) << desired_value(motors[0]));
	}
}

// Joint limits definitions
lib::MotorArray::value_type model::lower_joint_limit = 0;
lib::MotorArray::value_type model::upper_joint_limit = 2*M_PI/3;

void model::check_joints(const lib::JointArray & joints) const
{
	if (joints[0] < lower_joint_limit) {
		BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(1) << limit_type(LOWER_LIMIT) << desired_value(joints[0]));
	}
	if (joints[0] > upper_joint_limit) {
		BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(1) << limit_type(UPPER_LIMIT) << desired_value(joints[0]));
	}
}

void model::mp2i_transform(const lib::MotorArray & motors, lib::JointArray & joints)
{
	joints[0] = motors[0]/i2m_ratio;
}

void model::i2mp_transform(lib::MotorArray & motors, const lib::JointArray & joints)
{
	motors[0] = joints[0]*i2m_ratio;
}

} // namespace smb
} // namespace kinematic
} // namespace mrrocpp
