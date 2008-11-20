#include "ecp/irp6_on_track/ecp_g_pw_kolo.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_g_pw_kolo::ecp_g_pw_kolo (ecp_task& _ecp_task)
        : ecp_generator (_ecp_task)
{
    delta_y = 0.005;
    r = 0.28;
}

bool ecp_g_pw_kolo::first_step()
{

    //Tak by³o w teach_in_generatorze.
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

    step_no=1;

    printf("fs\n");

    return true;
}

bool ecp_g_pw_kolo::next_step()
{
    printf("ss\n");
    double time; //Czas ruchu.

    /* W tym kroku doje¿d¿am koncówk± do ¶rodka okrêgu.*/
    if(step_no==1)
    {
        next_position[0] = 0.899;		//x
        next_position[1] = y0 =-0.02;		//y
        next_position[2] = z0 =0.30;		//z
        next_position[3] = -0.115654;
        next_position[4] = 1.38944;
        next_position[5] = 2.35145;
        next_position[6] = 0.074;
        next_position[7] = 0;
        time = 4;
        step_no++;
    }
    /* W tym kroku ustawiam koñcówkê w pozycji startowej - na okrêgu. */
    else if(step_no==2)
    {
        next_position[0] = 0.899;
        next_position[1] -= r;
        y = next_position[1];
        next_position[2] = 0.30;
        next_position[3] = -0.115654;
        next_position[4] = 1.38944;
        next_position[5] = 2.35145;
        next_position[6] = 0.074;
        next_position[7] = 0;
        time = 4;
        step_no++;
    }
    /* Wlasciwy ruch po okrêgu. W ka¿dym kolejnym kroku obliczam pozycje y i z. */
    else if(step_no < 2000)
    {
        //Sprawdzam, czy nie nalezy zmienic znaku przyrostu delta_y.
        if( next_position[1] + delta_y > y0 + r)
        {
            delta_y = -delta_y;
            y = y0+r;
            z = z0;
        }
        else if( next_position[1] + delta_y < y0 - r )
        {
            delta_y = -delta_y;
            y = y0-r;
            z = z0;
        }
        else
        {
            //Obliczam nowe polozenia y i z.
            y += delta_y;
            if (delta_y >= 0)
                z = z0 + sqrt(r*r - (y -y0)*(y -y0));
            else
                z = z0 - sqrt(r*r - (y -y0)*(y -y0));
        }
        next_position[0] = 0.899;
        next_position[1] = y;
        next_position[2] = z;
        next_position[3] = -0.115654;
        next_position[4] = 1.38944;
        next_position[5] = 2.35145;
        next_position[6] = 0.074;
        next_position[7] = 0;
        time = 0.0000001;
        step_no++;
    }
    else
        return false;

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
