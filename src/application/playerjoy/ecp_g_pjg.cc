#include <cstdio>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/playerjoy/ecp_g_pjg.h"
#include "application/playerjoy/ecp_mp_tr_player.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

playerjoy::playerjoy(common::task::task& _ecp_task, int step):
	generator (_ecp_task)
{
    step_no = step;
}

bool playerjoy::first_step ( )
{
    run_counter = 0;
    second_step = false;

    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;

    the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;

    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
     the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    //the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
    //the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;



    return true;
}

bool playerjoy::next_step ( )
{
    if (check_and_null_trigger())
    {

        return false;
    }



    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
    the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;


    transmitter_m[ecp_mp::transmitter::TRANSMITTER_PLAYER]->t_read(0);
    /*
    printf("%f %f 0x%04x\n",
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.px,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.py,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_joystick.buttons
          );
    */

    for (int i = 0; i < lib::MAX_SERVOS_NR; i++)
    {
        if (node_counter <= 2)
        {
            start_joint_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
        }
        else
        {
            if (transmitter_m[ecp_mp::transmitter::TRANSMITTER_PLAYER]->from_va.player_joystick.buttons & (1 << i))
            {
                start_joint_arm_coordinates[i] +=
                    transmitter_m[ecp_mp::transmitter::TRANSMITTER_PLAYER]->from_va.player_joystick.px*(16*(10*M_PI/180)/1000/4);
            }
        }
    }

    for (int i = 0; i < 8; i++)
    {
        the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = start_joint_arm_coordinates[i];
        //printf("%f ", the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]);
    }

    //printf("\n");



    return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
