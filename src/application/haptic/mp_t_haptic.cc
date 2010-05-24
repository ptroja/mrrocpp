// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "application/haptic/mp_g_haptic.h"
#include "application/haptic/mp_t_haptic.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task (lib::configurator &_config)
{
	return new haptic(_config);
}

haptic::haptic(lib::configurator &_config) : task(_config)
{
}

void haptic::configure_edp_force_sensor(bool configure_track, bool configure_postument)
{
    if (configure_track)
    {
        set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE, 0, "", 0, 1, lib::ROBOT_IRP6OT_M);
    }

    if (configure_postument)
    {
        set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE, 0, "", 0, 1, lib::ROBOT_IRP6P_M);
    }

    if ((configure_track)&&(!configure_postument))
    {
        run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
                (1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);
    }
    else if ((!configure_track)&&(configure_postument))
    {
        run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
                (1, 1, lib::ROBOT_IRP6P_M, lib::ROBOT_IRP6P_M);
    }
    else if ((configure_track)&&(configure_postument))
    {
        run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
                (2, 2, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6P_M,
                		lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6P_M );
    }
}


void haptic::main_task_algorithm(void)
{
	generator::haptic mp_h_gen(*this, 10);
   	mp_h_gen.robot_m = robot_m;


   	sr_ecp_msg->message("New series");
   	//pierwsza konfiguracja czujnikow
   	// wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
   	configure_edp_force_sensor(true, true);


   	// wlaczenie generatora transparentnego w obu robotach
   	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::ROBOT_IRP6OT_M);
   	set_next_ecps_state ((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::ROBOT_IRP6P_M);

   	// mp_h_gen.sensor_m = sensor_m;
   	mp_h_gen.configure(1, 0);
   	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
   	mp_h_gen.Move();


   	send_end_motion_to_ecps (2, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6P_M);
}


} // namespace task
} // namespace mp
} // namespace mrrocpp
