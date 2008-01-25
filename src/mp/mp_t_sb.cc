// ------------------------------------------------------------------------
//             mp_m_pr.cc - powielanie rysunku - wersja wielorobotowa
// 
//                      MASTER PROCESS (MP) - main()
// 
// 			 Ostatnia modyfikacja: 2005
// ------------------------------------------------------------------------


#include <unistd.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_force.h"
#include "mp/mp_t_sb.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

mp_task* return_created_mp_task (void)
{
	return new mp_task_sb();
}

// methods fo mp template to redefine in concete class
void mp_task_sb::task_initialization(void) 
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);
		
	sensor_m[SENSOR_FORCE_POSTUMENT] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);
		
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}
	
	usleep(1000*100);
	sr_ecp_msg->message("MP sb loaded");

};
 


void mp_task_sb::main_task_algorithm(void)
{

	break_state = false;
	 // printf("przed wait for start \n");
      // Oczekiwanie na zlecenie START od UI  
    	sr_ecp_msg->message("MP sb - wcisnij start");
      wait_for_start ();
	      // Wyslanie START do wszystkich ECP 
      start_all (robot_m);

      for (;;) {  // Wewnetrzna petla    

       	// Zlecenie wykonania kolejnego makrokroku
		// printf("po start all \n");
		  for(;;) {
			sr_ecp_msg->message("Nowa seria");
	
			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}
			usleep(1000*1000);
			
		    	 mp_tff_nose_run_generator mp_tff_fr_gen(*this, 10); 
			 mp_tff_fr_gen.configure(1, 0);
			// ORYGINAL
			/*
			if (Move ( mp_tff_fr_gen)) {
		        	break_state = true;
		          break;
	          }
			*/

			// EXPERIMENTAL
			
			/*
		   	mp_tff_fr_gen.robot_m = robot_m;
		     mp_tff_fr_gen.sensor_m = sensor_m;
		
			
		
			add_gen (&mp_tff_fr_gen);
			
			if (scheduller_run ())
			{
			    	break_state = true;
		          break;
	          }
			*/	
	
	          clear_gen_list();
	        
	        
	          mp_tff_single_robot_nose_run_generator mp_tff_sr_fr_gen(*this, 10); 
    			mp_tff_sr_fr_gen.robot_m[ROBOT_IRP6_ON_TRACK] = robot_m[ROBOT_IRP6_ON_TRACK];
		     mp_tff_sr_fr_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
			
			add_gen (&mp_tff_sr_fr_gen);
	
		
			mp_tff_single_robot_nose_run_generator mp_tff_sr_fr_gen_p(*this, 10); 
    			mp_tff_sr_fr_gen_p.robot_m[ROBOT_IRP6_POSTUMENT] = robot_m[ROBOT_IRP6_POSTUMENT];
		     mp_tff_sr_fr_gen_p.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];
					
			add_gen (&mp_tff_sr_fr_gen_p);
			
			if (scheduller_run ())
			{
			    	break_state = true;
		          break;
	          }
	
			/*
			mp_teach_in_generator mp_ti1_gen(*this);
//			std::cout << "za mp_teach_in_generator run 1" << std::endl;
			mp_load_file_with_path (mp_ti1_gen, "../trj/irp6ot_tr_1.trj", 1);
//			std::cout << "za mp_teach_in_generator run 2" << std::endl;
		
			mp_ti1_gen.initiate_pose_list();
//						std::cout << "za mp_teach_in_generator run 3" << std::endl;
   			gen_class mp_ti1_gen_gen_gen(&mp_ti1_gen);
	
			mp_ti1_gen_gen_gen.add_robot(robot_m->E_ptr);
	
			mp_ti1_gen_gen_gen.add_controlled_sensor((sensor_m)->E_ptr);
					
			add_gen (&mp_ti1_gen_gen_gen);
	//		std::cout << "za mp_teach_in_generator run 4" << std::endl;
			
			mp_tff_single_robot_nose_run_generator mp_tff_sr_fr_gen_p(*this, 10); 
    			gen_class mp_tff_sr_fr_gen_gen_p(&mp_tff_sr_fr_gen_p);
	
			mp_tff_sr_fr_gen_gen_p.add_robot(robot_m->next->E_ptr);
	
			mp_tff_sr_fr_gen_gen_p.add_controlled_sensor((sensor_m)->next->E_ptr);
					
			add_gen (&mp_tff_sr_fr_gen_gen_p);
			
*/			
			/*
			if (Move (robot_m->next, (sensor_m), mp_ti1_gen)){
				break_state = true;
		   		break;
		    }
			*/
/*
			if (scheduller_run ())
			{
			    	break_state = true;
		          break;
	          }
			if (mp_ti1_gen_gen_gen.phase == GS_FINISHED) std::cout << "za GS_FINISHED run 1" << std::endl;
	          	std::cout << "za scheduller run 1" << mp_ti1_gen_gen_gen.phase << std::endl;
	         
	         	if (scheduller_run ())
			{
			    	break_state = true;
		          break;
	          }
*/	          
	     	std::cout << "za scheduller run 2" << std::endl;
	     	/*
	     	mp_tff_sr_fr_gen_gen_p.re_run();
	         	if (scheduller_run ())
			{
			    	break_state = true;
		          break;
	          }
	          
	     	std::cout << "za scheduller run 3" << std::endl;
	  */   	
			// END EXPERIMENTAL
			
			
					/*
		    	mp_tff_rubik_face_rotate_generator mp_tff_fr_gen(*this, 10); 
			mp_tff_fr_gen.configure(0,90.0);
			if (Move ( mp_tff_fr_gen)) {
		        	break_state = true;
		          break;
	          }
	          */

	 	}
	 	if (break_state) break;
		
        // Oczekiwanie na STOP od UI
  //          printf("w mp przed wait for stop\n");
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
        break; 
      } // koniec: for(;;) - wewnetrzna petla
};
