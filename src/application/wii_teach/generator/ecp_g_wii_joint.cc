#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "application/wii_teach/generator/ecp_g_wii_joint.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_joint::wii_joint (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : wii(_ecp_task,_wiimote)
{
    int i;
    for(i = 0;i<7;++i)
    {
        multipliers[i] = 0.003;
        maxChange[i] = 0.0001;
    }
}

void wii_joint::set_position(void)
{
    the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = nextChange[0];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = nextChange[1];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = nextChange[2];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = nextChange[3];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = nextChange[4];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = nextChange[5];
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6] = 0;
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[7] = 0;
    the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = nextChange[6];
}

bool wii_joint::first_step()
{
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    releasedA = false;
    stop = false;

    return true;
}

void wii_joint::preset_position(void)
{
    int i;
    for(i = 0;i < 7;++i)
    {
        requestedChange[i] = 0;
    }
    currentGripperValue = 0;//the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate;
}


}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp