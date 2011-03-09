
#include <cmath>

#include "base/lib/com_buf.h"
#include "robot/polycrank/kinematic_model_polycrank.h"

namespace mrrocpp {
namespace kinematics {
namespace polycrank {

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

void model::check_motor_position(const lib::MotorArray & motor_position) const
{
}

void model::check_joints(const lib::JointArray & q) const
{
}

void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	local_current_joints[0] = local_current_motor_pos[0] / motor_to_intext_ratio;
	local_current_joints[1] = local_current_motor_pos[1] / motor_to_intext_ratio;
	local_current_joints[2] = local_current_motor_pos[2] / motor_to_intext_ratio;
	local_current_joints[3] = local_current_motor_pos[3] / motor_to_intext_ratio;
	local_current_joints[4] = local_current_motor_pos[4] / motor_to_intext_ratio;
	local_current_joints[5] = local_current_motor_pos[5] / motor_to_intext_ratio;
	local_current_joints[6] = local_current_motor_pos[6] / motor_to_intext_ratio;
}

void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	local_desired_motor_pos_new[0] = local_desired_joints[0] * motor_to_intext_ratio;
	local_desired_motor_pos_new[1] = local_desired_joints[1] * motor_to_intext_ratio;
	local_desired_motor_pos_new[2] = local_desired_joints[2] * motor_to_intext_ratio;
	local_desired_motor_pos_new[3] = local_desired_joints[3] * motor_to_intext_ratio;
	local_desired_motor_pos_new[4] = local_desired_joints[4] * motor_to_intext_ratio;
	local_desired_motor_pos_new[5] = local_desired_joints[5] * motor_to_intext_ratio;
	local_desired_motor_pos_new[6] = local_desired_joints[6] * motor_to_intext_ratio;
}

} // namespace polycrank
} // namespace kinematic
} // namespace mrrocpp

