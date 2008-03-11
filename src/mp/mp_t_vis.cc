// ------------------------------------------------------------------------
// 
//                      MASTER PROCESS (MP) - main()
// 
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_vis.h"
#include "mp/mp_g_vis.h"
#include "mp/mp_g_force.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_vis(_config);
}

mp_task_vis::mp_task_vis(configurator &_config) : mp_task(_config)
{
}

// methods fo mp template to redefine in concete class
void mp_task_vis::task_initialization(void) 
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);
		
	sensor_m[SENSOR_CAMERA_SA] = 
		new ecp_mp_vis_sensor (SENSOR_CAMERA_SA, "[vsp_vis]", *this);
	
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	
	usleep(1000*100);
	sr_ecp_msg->message("MP vis loaded");
};
 

void mp_task_vis::main_task_algorithm(void)
{
	break_state = false;

	mp_seven_eye_generator eyegen(*this, 4);
	eyegen.robot_m[ROBOT_IRP6_ON_TRACK] = robot_m[ROBOT_IRP6_ON_TRACK]; 
    	eyegen.sensor_m[SENSOR_CAMERA_SA] = sensor_m[SENSOR_CAMERA_SA];


	//mp_seven_eye_generator ynrlg(*this, ROBOT_IRP6_ON_TRACK, 4);

	 // printf("przed wait for start \n");
      // Oczekiwanie na zlecenie START od UI  
    	sr_ecp_msg->message("gimme this stupid cube");
      wait_for_start ();
	      // Wyslanie START do wszystkich ECP 
      
//start_all (robot_m);
      for (;;) {  // Wewnetrzna petla    
	 //mp_seven_eye_generator ynrlg(*this, 4);
		start_all (robot_m);
       	// Zlecenie wykonania kolejnego makrokroku
		 printf("po start all \n");
		  for(;;) {
			sr_ecp_msg->message("New loop");

	 		//mp_seven_eye_generator eyegen(*this, 4);
			//eyegen.robot_m = robot_m;
			//eyegen.sensor_m = sensor_m;
			//po cholere biasujemy jeszcze raz te czujniki i co to w ogole oznacza???
		  	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}
			
			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_1.trj", 1, ROBOT_IRP6_ON_TRACK)) 
			{  	
				break_state = true;
		          break; 
			}
			sr_ecp_msg->message("after genload");
			
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
 				(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK)) 
			{  	
			sr_ecp_msg->message("if genstart");
				break_state = true;
		          break; 
			}
			sr_ecp_msg->message("after genstart");
			
			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_2.trj", 1, ROBOT_IRP6_ON_TRACK)) 
			{  	
				break_state = true;
		          break; 
			}
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
 				(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK)) 
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
		
			
			// opcjonalne serwo wizyjne
			//if (mode)
			//{
		
			if (eyegen.Move() )
			{  	
				break_state = true;
	          	break; 
			}
			//}
			
			if (send_end_motion_to_ecps (1, ROBOT_IRP6_ON_TRACK))
			{  	
				break_state = true;
		          break; 
			}
			
			// wlaczenie generatora zacisku na kostce w robocie irp6ot
    			if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_1, "", 1, ROBOT_IRP6_ON_TRACK)) 
			{  	
				break_state = true;
		          break; 
			}
			// uruchomienie generatora empty_gen
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		 		(1, 1, ROBOT_IRP6_ON_TRACK, 	ROBOT_IRP6_ON_TRACK)) 
		    	{  	
				break_state = true;
		          break; 
			}
			// wlaczenie generatora zacisku na kostce w robocie irp6ot
		    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_2, "", 1, ROBOT_IRP6_ON_TRACK)) 
			{  	
				break_state = true;
		          break; 
			}
			// uruchomienie generatora empty_gen
		   	if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		 		(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK))
			{  	
				break_state = true;
		          break; 
			}
	     
	 	}
	 	
		if (break_state) break;
        // Oczekiwanie na STOP od UI
            printf("w mp przed wait for stop\n");
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	 terminate_all (robot_m);  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        //terminate_all (robot_m);
        break; 
      } // koniec: for(;;) - wewnetrzna petla
};
