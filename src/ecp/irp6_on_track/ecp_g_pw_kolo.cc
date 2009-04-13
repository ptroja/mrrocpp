#include "ecp/irp6_on_track/ecp_g_pw_kolo.h"

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

pw_kolo::pw_kolo (common::task::base& _ecp_task)
        : base (_ecp_task)
{
}

bool pw_kolo::first_step()
{

    //Tak bylo w teach_in_generatorze.
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

    r = 0.20;
    prev_rad = 0; // zaczynam od kata 0;
    d_rad = 0.5*DEG;

    step_no=1;

    return true;
}

bool pw_kolo::next_step()
{
    double time; //Czas ruchu.
//    double current_rad; //Aktualne polozenie w radianach.

    /* W tym kroku dojezdzam koncowka do srodka okregu.*/
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
        time = 10;
        step_no++;
    }
    /* W tym kroku ustawiam koncowke w pozycji startowej - na okregu. */
    else if(step_no==2)
    {
        next_position[0] = 0.899;
        next_position[1] += r;
        next_position[2] = 0.30;
        next_position[3] = -0.115654;
        next_position[4] = 1.38944;
        next_position[5] = 2.35145;
        next_position[6] = 0.074;
        next_position[7] = 0;
        time = 10;
        step_no++;
    }
    /* Wlasciwy ruch po okregu. W kazdym kolejnym kroku obliczam pozycje y i z. */
    else if(step_no < 1000)
    {
        next_position[0] = 0.899;
    	next_position[1] = cos( prev_rad + d_rad )*r + y0;
    	next_position[2] = sin( prev_rad + d_rad )*r + z0;
        next_position[3] = -0.115654;
        next_position[4] = 1.38944;
        next_position[5] = 2.35145;
        next_position[6] = 0.074;
        next_position[7] = 0;
        time = 0.025;
        prev_rad += d_rad;
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

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


