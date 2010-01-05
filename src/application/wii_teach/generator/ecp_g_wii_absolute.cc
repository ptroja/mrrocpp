#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "application/wii_teach/generator/ecp_g_wii_absolute.h"
#include "ecp_g_wii.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_absolute::wii_absolute (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : wii(_ecp_task,_wiimote)
{
    int i;
    for(i = 0;i<NO_OF_DEGREES;++i)
    {
        multipliers[i] = 0.003;
        maxChange[i] = 0.0001;
    }
}

void wii_absolute::set_position(void)
{
    char buffer[200];
    double rotation[3][3];
    double translation[3];

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

    homog_matrix.set_from_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
    homog_matrix.get_translation_vector(translation);
    homog_matrix.get_rotation_matrix(rotation);

    sr_ecp_msg.message("=======");
    sprintf(buffer,"Rotation:\n%.4f %.4f %.4f\n%.4f %.4f %.4f\n%.4f %.4f %.4f",rotation[0][0],rotation[1][0],rotation[2][0],rotation[0][1],rotation[1][1],rotation[2][1],rotation[0][2],rotation[1][2],rotation[2][2]);
    sr_ecp_msg.message(buffer);

    sprintf(buffer,"Translation:\n%.4f %.4f %.4f",translation[0],translation[1],translation[2]);
    sr_ecp_msg.message(buffer);

    

//    translation[0] += nextChange[0];
//    translation[1] += nextChange[1];
//    translation[2] += nextChange[2];
//
//    rotation[0][0] += cos(nextChange[3])*cos(nextChange[4]);
//    rotation[1][0] += sin(nextChange[3])*cos(nextChange[4]);
//    rotation[2][0] += -sin(nextChange[4]);
//    rotation[0][1] += cos(nextChange[3])*sin(nextChange[4])*sin(nextChange[5])-sin(nextChange[3])*cos(nextChange[5]);
//    rotation[1][1] += sin(nextChange[3])*sin(nextChange[4])*sin(nextChange[5])+cos(nextChange[3])*cos(nextChange[5]);
//    rotation[2][1] += cos(nextChange[4])*sin(nextChange[5]);
//    rotation[0][2] += cos(nextChange[3])*sin(nextChange[4])*cos(nextChange[5])+sin(nextChange[3])*sin(nextChange[5]);
//    rotation[1][2] += sin(nextChange[3])*sin(nextChange[4])*cos(nextChange[5])-cos(nextChange[3])*sin(nextChange[5]);
//    rotation[2][2] += cos(nextChange[4])*cos(nextChange[5]);

    homog_matrix.set_translation_vector(translation);
    homog_matrix.set_rotation_matrix(rotation);

    homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

    the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = 0;//currentGripperValue + nextChange[6];
}


bool wii_absolute::first_step()
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

void wii_absolute::preset_position(void)
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
