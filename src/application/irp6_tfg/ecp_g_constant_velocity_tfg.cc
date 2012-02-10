/**
 * @file
 * @brief Contains definitions of the methods of constant_velocity class.
 * @author rtulwin
 * @ingroup generators
 */

#include "ecp_g_constant_velocity_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace generator {

using namespace std;

constant_velocity::constant_velocity(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
		common::generator::constant_velocity(_ecp_task, pose_spec, axes_num)
{
	set_debug(true);
}

void constant_velocity::conditional_execution()
{

	reset();
	std::vector <double> pos(1, ecp_t.mp_command.ecp_next_state.sg_buf.get <double>());
	std::vector <double> joint_velocity(1, 0.003);

	set_joint_velocity_vector(joint_velocity);

	switch ((lib::MOTION_TYPE) ecp_t.mp_command.ecp_next_state.variant)
	{
		case lib::RELATIVE:
			set_relative();
			load_relative_joint_trajectory_pose(pos);
			break;
		case lib::ABSOLUTE:
			set_absolute();
			load_absolute_joint_trajectory_pose(pos);
			break;
		default:
			break;
	}

	calculate_interpolate();
	Move();
}

//--------------- METHODS USED TO LOAD POSES END ----------------

}// namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
