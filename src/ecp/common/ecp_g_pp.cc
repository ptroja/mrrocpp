// -------------------------------------------------------------------------
//                            ecp_gen_playerjoy.cc
//            Effector Control Process (lib::ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include "ecp_mp/ecp_mp_s_pp.h"
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_pp.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {



progpanel::progpanel(common::task::task& _ecp_task, int step):
	generator (_ecp_task)
{
    step_no = step;
}


bool progpanel::first_step ( )
{
    run_counter = 0;
    second_step = false;

    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->ecp_command.instruction.set_type = ARM_DV;

    the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;

    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
     the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
    the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;



    return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool progpanel::next_step ( )
{
    // zmienne wykorzystywane przy rysowaniu
    if (check_and_null_trigger())
    {
        return false;
    }


    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.get_type = NOTHING_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;


    ecp_mp::sensor::pp* pps = (ecp_mp::sensor::pp*)(sensor_m[lib::SENSOR_PP]);
    // Pobranie ostatniego odczytu z czujnika sily.
    //	pps->initiate_reading();
    //	pps->get_reading();


    //    transmitter_m[TRANSMITTER_PLAYER]->t_read(0);
    /*
    printf("%f %f 0x%04x\n",
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.px,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.py,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.buttons
          );
    */

    if (node_counter <= 2)
    {
        memcpy(start_joint_arm_coordinates, the_robot->EDP_data.current_joint_arm_coordinates, 8*(sizeof(double)));
    }
    else
    {
        //printf ("Active_motors = %d\n", pps->image.sensor_union.pp.active_motors);
        switch (pps->image.sensor_union.pp.active_motors)
        {
        case 0:
            start_joint_arm_coordinates[0] += pps->image.sensor_union.pp.joy[0];
            start_joint_arm_coordinates[1] += pps->image.sensor_union.pp.joy[1];
            start_joint_arm_coordinates[2] += pps->image.sensor_union.pp.joy[2];
            break;

        case 1:
            start_joint_arm_coordinates[3] += pps->image.sensor_union.pp.joy[0];
            start_joint_arm_coordinates[4] += pps->image.sensor_union.pp.joy[1];
            start_joint_arm_coordinates[5] += pps->image.sensor_union.pp.joy[2];
            break;

        case 2:
            start_joint_arm_coordinates[6] += pps->image.sensor_union.pp.joy[0];
            start_joint_arm_coordinates[7] += pps->image.sensor_union.pp.joy[1];
            break;

        default:
            break;
        }
    }

    memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, start_joint_arm_coordinates, 8*(sizeof(double)));

    /*    for (int i = 0; i < 8; i++)
    		printf("%f ", the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]);
     
        printf("\n");
    */


    return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
