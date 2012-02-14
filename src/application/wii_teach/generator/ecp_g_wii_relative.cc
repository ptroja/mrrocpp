#include <cmath>

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "application/wii_teach/generator/ecp_g_wii_relative.h"
#include "ecp_g_wii.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

wii_relative::wii_relative(common::task::task& _ecp_task, ecp_mp::sensor::wiimote* _wiimote) :
		wii(_ecp_task, _wiimote)
{
	char buffer[100];

	for (int i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		sprintf(buffer, "relative_multiplier_%d", i);
		multipliers[i] = ecp_t.config.value <double>(buffer);
		sprintf(buffer, "relative_max_change_%d", i);
		maxChange[i] = ecp_t.config.value <double>(buffer);
	}
}

void wii_relative::set_position(bool changed)
{
	//   char buffer[200];

	the_robot->ecp_command.instruction_type = lib::SET_GET;

	homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(nextChange[2], nextChange[0], nextChange[1], nextChange[4], nextChange[6], nextChange[5]));

	the_robot->ecp_command.arm.pf_def.arm_frame = homog_matrix;
}

bool wii_relative::first_step()
{
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 25;
	the_robot->ecp_command.value_in_step_no = 22;

	releasedA = false;
	stop = false;

	return true;
}

void wii_relative::preset_position(void)
{
	int i;
	for (i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		requestedChange[i] = 0;
	}
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
