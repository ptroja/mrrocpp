/*!
 * @file
 * @brief File containing methods of the kinematic model for the SwarmItFix agent's mobile base class.
 *
 * @author tkornuta
 * @date 2010.02.01
 *
 * @ingroup KINEMATICS SIF_KINEMATICS smb
 */

#include "base/lib/com_buf.h"
#include "robot/smb/kinematic_model_smb.h"
#include "exceptions.h"

using namespace mrrocpp::edp::exception;

namespace mrrocpp {
namespace kinematics {
namespace smb {

//! Parameters for conversion for rotational DOFs are:
//! * The encoder has 2400 CPT (Counts per turn).
//! * The gear ratio is 50.
//! * Motor to motor radio is ??? // TODO: CHECK!!
const double rotational_legs_mp2i_ratio = -M_PI / (2400 * 50 * 2.5);

//! Parameters for conversion for rotational DOFs are:
//! * The encoder has 2400 CPT (Counts per turn).
//! * The gear ratio is 50.
const double rotational_spkm_mp2i_ratio = -M_PI / (2400 * 50);

//! Initialization of motor to internal ratios.
const double model::mp2i_ratios[mrrocpp::lib::smb::NUM_OF_SERVOS] = { rotational_legs_mp2i_ratio, rotational_spkm_mp2i_ratio };

//! Initialization of parameters describing the synchronisation position  (in motor increments).
const int32_t model::synchro_motor_positions[mrrocpp::lib::smb::NUM_OF_SERVOS] = { 0, -11970 };

//! Initialization of upper motors limits for PKM.
const int32_t model::upper_pkm_motor_pos_limits = { 120000 };

//! Initialization of lower motors limits for PKM.
const int32_t model::lower_pkm_motor_pos_limits = { -120000 };


model::model(void)
{
	// Set label.
	set_kinematic_model_label("Switching to basic kinematic model");

	// Set kinematics parameters.
	set_kinematic_parameters();
}

void model::set_kinematic_parameters(void)
{
}

void model::check_motor_position(const lib::MotorArray & motor_position) const
{
	// Check upper and lower limits only for the motor rotating the PKM.
	if (motor_position[1] > upper_pkm_motor_pos_limits)
		BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(1) << limit_type(UPPER_LIMIT) << desired_value(motor_position[1]));
	else if (motor_position[1] < lower_pkm_motor_pos_limits)
		BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(1) << limit_type(LOWER_LIMIT) << desired_value(motor_position[1]));

	// The motor rotating legs doesn't have limits.
}

void model::check_joints(const lib::JointArray & q) const
{
	//This method is empty, because only motor position constraints are required.
}

void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	// Compute desired motor positions for both axes.
	for (int i = 0; i < 2; ++i) {
		local_current_joints[i] = (local_current_motor_pos[i] - synchro_motor_positions[i]) * mp2i_ratios[i];
		//local_current_motor_pos[i] * mp2i_ratios[i];

	}
}

void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{

	// Compute desired motor positions for both axes.
	for (int i = 0; i < 2; ++i) {
		local_desired_motor_pos_new[i] = (local_desired_joints[i] / mp2i_ratios[i]) + synchro_motor_positions[i];
		//local_desired_joints[i] / mp2i_ratios[i];
	}

}

} // namespace smb
} // namespace kinematic
} // namespace mrrocpp

