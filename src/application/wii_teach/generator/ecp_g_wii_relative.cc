#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "application/wii_teach/generator/ecp_g_wii_relative.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_relative::wii_relative (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : wii(_ecp_task,_wiimote)
{
    int i;
    for(i = 0;i<7;++i)
    {
        multipliers[i] = 0.003;
        maxChange[i] = 0.0001;
    }
}

bool wii_relative::first_step()
{
	/*
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    step_no = 0;
    releasedA = false;
    stop = false;

    return true;
    */
}

void wii_relative::preset_position(void)
{
    currentValue[0] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0];
    currentValue[1] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1];
    currentValue[2] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2];
    currentValue[3] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3];
    currentValue[4] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4];
    currentValue[5] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5];
    currentValue[6] = the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate;
}


}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
