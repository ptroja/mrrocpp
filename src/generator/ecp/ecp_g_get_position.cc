/**
 * @file
 * @brief Contains definitions of the methods of get_position class.
 * @author rtulwin
 * @ingroup generators
 */

#include "base/ecp/ecp_robot.h"
#include "ecp_g_get_position.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;

get_position::get_position(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
	generator(_ecp_task)
{
	position = vector <double> ();
	this->axes_num = axes_num;
	this->pose_spec = pose_spec;
}

get_position::~get_position()
{

}

bool get_position::first_step()
{
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE; //aqui siempre ABSOLUTE, RELATIVE makes no sense here
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;

	switch (pose_spec)
	{
		case lib::ECP_XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
			break;
		case lib::ECP_XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
			break;
		case lib::ECP_MOTOR:
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::ECP_JOINT:
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
		default:
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}
	return true;
}

bool get_position::next_step()
{
	if (pose_spec == lib::ECP_XYZ_ANGLE_AXIS || pose_spec == lib::ECP_XYZ_EULER_ZYZ) {

		lib::Homog_matrix actual_position_matrix;
		actual_position_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

		if (pose_spec == lib::ECP_XYZ_ANGLE_AXIS) {
			lib::Xyz_Angle_Axis_vector angle_axis_vector;
			actual_position_matrix.get_xyz_angle_axis(angle_axis_vector);
			angle_axis_vector.to_vector(position);

		} else if (pose_spec == lib::ECP_XYZ_EULER_ZYZ) {
			lib::Xyz_Euler_Zyz_vector euler_vector;
			actual_position_matrix.get_xyz_euler_zyz(euler_vector);
			euler_vector.to_vector(position);

		} else {
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}

	} else if (pose_spec == lib::ECP_JOINT || pose_spec == lib::ECP_MOTOR) {
		for (int i = 0; i < axes_num; i++) {
			position.push_back(the_robot->reply_package.arm.pf_def.arm_coordinates[i]);
		}
	} else {
		throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}
	return false;
}

vector <double> get_position::get_position_vector()
{
	return position;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
