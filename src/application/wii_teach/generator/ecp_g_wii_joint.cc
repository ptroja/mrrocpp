#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "math.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "application/wii_teach/generator/ecp_g_wii_joint.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

wii_joint::wii_joint (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : wii(_ecp_task,_wiimote)
{
    int i;
    char buffer[100];

    for(i = 0;i<MAX_NO_OF_DEGREES;++i)
    {
        sprintf(buffer,"joint_multiplier_%d",i);
        multipliers[i] = ecp_t.config.value<double>(buffer);
        sprintf(buffer,"joint_max_change_%d",i);
        maxChange[i] = ecp_t.config.value<double>(buffer);
    }
}

void wii_joint::set_position(void)
{
    the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = nextChange[0];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = nextChange[1];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = nextChange[2];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = nextChange[3];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = nextChange[4];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = nextChange[5];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6] = nextChange[6];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[7] = nextChange[7];
}

bool wii_joint::first_step()
{
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 25;
    the_robot->ecp_command.instruction.value_in_step_no = 22;

    releasedA = false;
    stop = false;

    return true;
}

void wii_joint::preset_position(void)
{
    int i;
    for(i = 0;i < MAX_NO_OF_DEGREES;++i)
    {
        requestedChange[i] = 0;
    }
}


}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
