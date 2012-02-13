#include <cmath>

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "application/wii_teach/generator/ecp_g_wii_absolute.h"
#include "ecp_g_wii.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

wii_absolute::wii_absolute(common::task::task& _ecp_task, ecp_mp::sensor::wiimote* _wiimote) :
		wii(_ecp_task, _wiimote)
{
	int i;
	char buffer[100];

	for (i = 0; i < MAX_NO_OF_DEGREES; ++i) {
		sprintf(buffer, "absolute_multiplier_%d", i);
		multipliers[i] = ecp_t.config.value <double>(buffer);
		sprintf(buffer, "absolute_max_change_%d", i);
		maxChange[i] = ecp_t.config.value <double>(buffer);
	}
}

void wii_absolute::set_position(bool changed)
{
	double rotation[3][3];
	double translation[3];
	double old_translation[3];

	if (the_robot->ecp_command.instruction_type == lib::GET) {
		homog_matrix = the_robot->ecp_command.arm.pf_def.arm_frame;
	} else {
		homog_matrix = the_robot->reply_package.arm.pf_def.arm_frame;
	}

	the_robot->ecp_command.instruction_type = lib::SET_GET;

	homog_matrix.get_translation_vector(old_translation);

	translation[0] = nextChange[1];
	translation[1] = nextChange[0];
	translation[2] = nextChange[2];

	rotation[0][0] = cos(nextChange[6]) * cos(nextChange[4]);
	rotation[1][0] = sin(nextChange[6]) * cos(nextChange[4]);
	rotation[2][0] = -sin(nextChange[4]);
	rotation[0][1] = cos(nextChange[6]) * sin(nextChange[4]) * sin(nextChange[5])
			- sin(nextChange[6]) * cos(nextChange[5]);
	rotation[1][1] = sin(nextChange[6]) * sin(nextChange[4]) * sin(nextChange[5])
			+ cos(nextChange[6]) * cos(nextChange[5]);
	rotation[2][1] = cos(nextChange[4]) * sin(nextChange[5]);
	rotation[0][2] = cos(nextChange[6]) * sin(nextChange[4]) * cos(nextChange[5])
			+ sin(nextChange[6]) * sin(nextChange[5]);
	rotation[1][2] = sin(nextChange[6]) * sin(nextChange[4]) * cos(nextChange[5])
			- cos(nextChange[6]) * sin(nextChange[5]);
	rotation[2][2] = cos(nextChange[4]) * cos(nextChange[5]);

	rotation_matrix.set_rotation_matrix(rotation);

	homog_matrix = rotation_matrix * homog_matrix;
	old_translation[0] += translation[0];
	old_translation[1] += translation[1];
	old_translation[2] += translation[2];
	homog_matrix.set_translation_vector(old_translation);

	the_robot->ecp_command.arm.pf_def.arm_frame = homog_matrix;
}

bool wii_absolute::first_step()
{
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 25;
	the_robot->ecp_command.value_in_step_no = 22;

	releasedA = false;
	stop = false;

	return true;
}

void wii_absolute::preset_position(void)
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
