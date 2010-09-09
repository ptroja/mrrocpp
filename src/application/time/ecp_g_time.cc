// -------------------------------------------------------------------------
//                            ecp_gen_time.cc
//            Effector Control Process (lib::ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp/ecp_robot.h"
#include "base/lib/sr/srlib.h"
#include "ecp_g_time.h"

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

time::time(common::task::task& _ecp_task, int step):
	generator (_ecp_task)
{
    step_no = step;
}


bool time::first_step ( )
{
    run_counter = 0;
    second_step = false;

    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;

    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;

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
bool time::next_step ( )
{
    // zmienne wykorzystywane przy rysowaniu
    if (check_and_null_trigger())
    {
        return false;
    }

    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
    the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;


    //transmitter_m[TRANSMITTER_PLAYER]->t_read(0);

#if 0
    printf("%lu.%09lu\n",
           sensor_m[lib::SENSOR_TIME]->image.sensor_union.time.ts.tv_sec,
           sensor_m[lib::SENSOR_TIME]->image.sensor_union.time.ts.tv_nsec
          );
#endif

    if (node_counter <= 2)
    {
        for (int i = 0; i < 8; i++)
            start_joint_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            //start_joint_arm_coordinates[i] +=
        }
    }

    for (int i = 0; i < 8; i++)
    {
        the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = start_joint_arm_coordinates[i];
    }

    //printf("\n");

    return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
