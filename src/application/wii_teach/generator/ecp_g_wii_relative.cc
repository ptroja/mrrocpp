#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "application/wii_teach/generator/ecp_g_wii_relative.h"
#include "ecp_g_wii.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_relative::wii_relative (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : wii(_ecp_task,_wiimote)
{
    int i;
    char buffer[100];
    
    for(i = 0;i<NO_OF_DEGREES;++i)
    {
        sprintf(buffer,"relative_multiplier_%d",i);
        multipliers[i] = ecp_t.config.value<double>(buffer);
        sprintf(buffer,"relative_max_change_%d",i);
        maxChange[i] = ecp_t.config.value<double>(buffer);
    }
}

void wii_relative::set_position(void)
{
    char buffer[200];

    the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    homog_matrix.set_from_xyz_angle_axis(
        nextChange[3],
        nextChange[4],
        nextChange[5],
        nextChange[0],
        nextChange[1],
        nextChange[2]
    );

    homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

    the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = 0;//currentGripperValue + nextChange[6];
}


bool wii_relative::first_step()
{
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
    the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 8;

    releasedA = false;
    stop = false;

    return true;
}

void wii_relative::preset_position(void)
{
    int i;
    for(i = 0;i < NO_OF_DEGREES;++i)
    {
        requestedChange[i] = 0;
    }
    currentGripperValue = 0;//the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate;
}


}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
