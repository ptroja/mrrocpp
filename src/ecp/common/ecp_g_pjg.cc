#include <stdio.h>
#include <math.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_pjg.h"
#include "ecp_mp/ecp_mp_tr_player.h"

playerjoy_generator::playerjoy_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
}

bool playerjoy_generator::first_step ( )
{
    run_counter = 0;
    second_step = false;

    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV;

    the_robot->EDP_data.set_arm_type = JOINT;
    the_robot->EDP_data.get_arm_type = JOINT;

    the_robot->EDP_data.motion_type = ABSOLUTE;
    //the_robot->EDP_data.motion_steps = td.internode_step_no;
    //the_robot->EDP_data.value_in_step_no = td.value_in_step_no;



    return true;
}

bool playerjoy_generator::next_step ( )
{
    if (ecp_t.pulse_check())
    {
        ecp_t.mp_buffer_receive_and_send ();
        return false;
    }



    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
    node_counter++;

    transmitter_m[TRANSMITTER_PLAYER]->t_read(0);
    /*
    printf("%f %f 0x%04x\n",
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.px,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.py,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.buttons
          );
    */

    for (int i = 0; i < MAX_SERVOS_NR; i++)
    {
        if (node_counter <= 2)
        {
            start_joint_arm_coordinates[i] = the_robot->EDP_data.current_joint_arm_coordinates[i];
        }
        else
        {
            if (transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.buttons & (1 << i))
            {
                start_joint_arm_coordinates[i] +=
                    transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.px*(16*(10*M_PI/180)/1000/4);
            }
        }
    }

    for (int i = 0; i < 8; i++)
    {
        the_robot->EDP_data.next_joint_arm_coordinates[i] = start_joint_arm_coordinates[i];
        //printf("%f ", the_robot->EDP_data.next_joint_arm_coordinates[i]);
    }

    //printf("\n");



    return true;
}
