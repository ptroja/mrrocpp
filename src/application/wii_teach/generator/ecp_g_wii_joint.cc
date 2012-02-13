#include <cmath>

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "application/wii_teach/generator/ecp_g_wii_joint.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

wii_joint::wii_joint(common::task::task& _ecp_task, ecp_mp::sensor::wiimote* _wiimote) :
		wii(_ecp_task, _wiimote)
{
	int i;
	char buffer[100];

	for (i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		sprintf(buffer, "joint_multiplier_%d", i);
		multipliers[i] = ecp_t.config.value <double>(buffer);
		sprintf(buffer, "joint_max_change_%d", i);
		maxChange[i] = ecp_t.config.value <double>(buffer);
	}
}

void wii_joint::set_position(bool changed)
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	for (int i = 0; i <= 7; ++i) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = nextChange[i];
	}
}

bool wii_joint::first_step()
{
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//    the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::JOINT;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 25;
	the_robot->ecp_command.value_in_step_no = 22;

	releasedA = false;
	stop = false;

	return true;
}

void wii_joint::preset_position(void)
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
