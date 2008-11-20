#include "ecp/irp6_on_track/ecp_g_pw_scena.h"


ecp_g_pw_scena::ecp_g_pw_scena (ecp_task& _ecp_task)
        : ecp_generator (_ecp_task)
{

}

bool ecp_g_pw_scena::first_step()
{
    the_robot->EDP_data.get_type = ARM_DV; // ARM
    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = MOTOR;
    the_robot->EDP_data.get_arm_type = MOTOR;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 6;

    //step_no=1;
}

bool ecp_g_pw_scena::next_step()
{
    double time; //Czas ruchu.


    printf("przed robotem\n");
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV; // ARM
    the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = (WORD) ceil(time/STEP);//ceil(tip.motion_time/STEP);
    the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps;

    memcpy(the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates, next_position,
           6*sizeof (double));
    the_robot->EDP_data.next_gripper_coordinate = next_position[6];
    printf("po robocie \n");
    return true;
}
