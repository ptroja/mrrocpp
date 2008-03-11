// -------------------------------------------------------------------------
//                              mp_t_haptic.cc
// 
// MP task for two robot haptic device
// 
// -------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_force.h"
#include "mp/mp_t_haptic.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_haptic(_config);
}

mp_task_haptic::mp_task_haptic(configurator &_config) : mp_task(_config)
{
}

// methods fo mp template to redefine in concete class
void mp_task_haptic::task_initialization(void) 
{
	sr_ecp_msg->message("MP haptic device loaded");
};



bool mp_task_haptic::configure_edp_force_sensor(bool configure_track, bool configure_postument)
{
    if (configure_track)
    {
        if (set_next_ecps_state ((int) ECP_GEN_BIAS_EDP_FORCE, 0, "", 1, ROBOT_IRP6_ON_TRACK))
        {
            return true;
        }
    }

    if (configure_postument)
    {
        if (set_next_ecps_state ((int) ECP_GEN_BIAS_EDP_FORCE, 0, "", 1, ROBOT_IRP6_POSTUMENT))
        {
            return true;
        }
    }

    if ((configure_track)&&(!configure_postument))
    {
        if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
                (1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK))
        {
            return true;
        }
    }
    else if ((!configure_track)&&(configure_postument))
    {
        if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
                (1, 1, ROBOT_IRP6_POSTUMENT, ROBOT_IRP6_POSTUMENT))
        {
            return true;
        }
    }
    else if ((configure_track)&&(configure_postument))
    {
        if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
                (2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT,
                 ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT ))
        {
            return true;
        }
    }
        return false;
}

 
void mp_task_haptic::main_task_algorithm(void)
{

	break_state = false;

  	mp_haptic_generator mp_h_gen(*this, 10); 
   	mp_h_gen.robot_m = robot_m;


	 // printf("przed wait for start \n");
      // Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP haptic device - press start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);

	for (;;)
	{  // Wewnetrzna petla

       	// Zlecenie wykonania kolejnego makrokroku
		// printf("po start all \n");
		 for(;;)
		{
			sr_ecp_msg->message("New series");
			    //pierwsza konfiguracja czujnikow
			    // wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
			    if (configure_edp_force_sensor(true, true))
			    {
			              	break_state = true;
		        	break;			    
			    }


			    // wlaczenie generatora transparentnego w obu robotach
			    if (set_next_ecps_state ((int) ECP_GEN_TRANSPARENT, (int) 0, "", 1, ROBOT_IRP6_ON_TRACK))
			    {
			              	break_state = true;
		        	break;
			    }
			    if (set_next_ecps_state ((int) ECP_GEN_TRANSPARENT, (int) 0, "", 1, ROBOT_IRP6_POSTUMENT))
			    {
			              	break_state = true;
		        	break;
			    }
			
			// mp_h_gen.sensor_m = sensor_m;
			mp_h_gen.configure(1, 0);
		    	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
			if (mp_h_gen.Move()) {
		        	break_state = true;
		        	break;
	         }

			
			    if (send_end_motion_to_ecps (2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT))
			    {
			              	break_state = true;
		        	break;
			    }






      	}
	 	
	 	
		if (break_state) 
		{
			break;
		}	
		
        // Oczekiwanie na STOP od UI
	  //          printf("w mp przed wait for stop\n");
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
        break; 
      } // koniec: for(;;) - wewnetrzna petla
}
