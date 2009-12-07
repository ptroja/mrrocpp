#include "ecp/irp6_on_track/ecp_g_pw_kolo.h"

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

pw_kolo::pw_kolo (common::task::task& _ecp_task)
        : generator (_ecp_task)
{
}

bool pw_kolo::first_step()
{

    //Tak bylo w teach_in_generatorze.
    the_robot->ecp_command.instruction.get_type = ARM_DV; // ARM

    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
    the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 6;

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
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DV; // ARM
    the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(time/STEP);//ceil(tip.motion_time/STEP);
    the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;

    memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, next_position,
           6*sizeof (double));
    the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = next_position[6];
    printf("po robocie \n");
    return true;
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


