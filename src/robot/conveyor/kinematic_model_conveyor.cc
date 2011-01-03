/*!
 * @file
 * @brief File containing the methods of the conveyor kinematic model.
 *
 * @author tkornuta
 * @date 24.02.2007
 *
 * @ingroup KINEMATICS CONVEYOR_KINEMATICS conveyor
 */

#include "robot/conveyor/kinematic_model_conveyor.h"

namespace mrrocpp {
namespace kinematics {
namespace conveyor {

model::model(void)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to standard kinematic model");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();
}

void model::set_kinematic_parameters(void)
{
	// Polozenie synchronizacji.
	synchro_motor_position = 0;
	// Stosunek polozenia walu silnika do polozenia we wsp. wewn (zewn) w metrach.
	motor_to_intext_ratio = 2250;
}

void model::check_motor_position(const lib::MotorArray & motor_position)
{
	return;
}

void model::check_joints(const lib::JointArray & q)
{
	return;
}

void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	local_current_joints[0] = local_current_motor_pos[0] / motor_to_intext_ratio;
}

void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	local_desired_motor_pos_new[0] = local_desired_joints[0] * motor_to_intext_ratio;
}

} // namespace conveyor
} // namespace kinematic
} // namespace mrrocpp

