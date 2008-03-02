// ------------------------------------------------------------------------
//             mp_m_pr.cc - powielanie rysunku - wersja wielorobotowa
// 
//                      MASTER PROCESS (MP) - main()
// 
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_force.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "mp/mp_t_pr.h"


mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_pr(_config);
}

mp_task_pr::mp_task_pr(configurator &_config) : mp_task(_config)
{
}

// methods for mp template to redefine in concrete class
void mp_task_pr::task_initialization(void) 
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
	sr_ecp_msg->message("MP pr loaded");

};
 

void mp_task_pr::main_task_algorithm(void)
{
	break_state = false;
    	mp_nose_run_force_generator mp_nrf_gen(*this, 8); 
    	mp_nrf_gen.robot_m = robot_m;
     mp_nrf_gen.sensor_m = sensor_m;
    	mp_drawing_teach_in_force_generator mp_dtif_gen(*this, 8);
    	mp_dtif_gen.robot_m = robot_m;
     mp_dtif_gen.sensor_m = sensor_m;
	 // printf("przed wait for start \n");
      // Oczekiwanie na zlecenie START od UI  
    	sr_ecp_msg->message("MP powielanie rysunku - wcisnij start");
      wait_for_start ();
	      // Wyslanie START do wszystkich ECP 
      start_all (robot_m);

      for (;;) {  // Wewnetrzna petla    

       	// Zlecenie wykonania kolejnego makrokroku
		// printf("po start all \n");
		  for(;;) {

			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}
			
	 		sr_ecp_msg->message("NOWA SERIA");
 			sr_ecp_msg->message("Wodzenie za nos do pozycji rozpoczecia nauki");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE MP trigger");
			
			if (Move ( mp_nrf_gen)){
		        	break_state = true;
		          break;
	          }

			if (choose_option ("1 - Load drawing, 2 - Learn drawing", 2) == OPTION_ONE)
			{
				sr_ecp_msg->message("Wczytywanie trajektorii");
   				mp_dtif_gen.load_file ();
			} else {
			
				sr_ecp_msg->message("Wodzenie za nos");
				sr_ecp_msg->message("Nastepny etap - nacisnij PULSE MP trigger");
				if (Move ( mp_nrf_gen)){
			        	break_state = true;
			          break;
		          }
			
				sr_ecp_msg->message("Uczenie trajektorii");
				sr_ecp_msg->message("Nastepny etap - nacisnij PULSE MP trigger");
			
				mp_dtif_gen.flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
				mp_dtif_gen.teach_or_move=YG_TEACH;
				if (Move ( mp_dtif_gen)){
			        	break_state = true;
			          break;
		          }
				
				sr_ecp_msg->message("Krotki ruch w gore");
				mp_short_move_up(*this);
				
				sr_ecp_msg->message("Wodzenie za nos");
				sr_ecp_msg->message("Nastepny etap - nacisnij PULSE MP trigger");
				if (Move ( mp_nrf_gen)){
			        	break_state = true;
			          break;
		          }
			}
			
			while (operator_reaction ("Reproduce drawing?"))
			{
				sr_ecp_msg->message("Wodzenie za nos do poczatku odtwarzania");
				sr_ecp_msg->message("Nastepny etap - nacisnij PULSE MP trigger");
				if (Move ( mp_nrf_gen)){
			        	break_state = true;
			          break;
		          }

				sr_ecp_msg->message("Odtwarzanie nauczonej trajektorii");
	
				mp_dtif_gen.teach_or_move=YG_MOVE;
				if (Move ( mp_dtif_gen)){
			        	break_state = true;
			          break;
		          }
			
				sr_ecp_msg->message("Krotki ruch w gore");
				mp_short_move_up(*this);

				sr_ecp_msg->message("Wodzenie za nos");
				sr_ecp_msg->message("Nastepny etap - nacisnij PULSE MP trigger");
				if (Move ( mp_nrf_gen)){
			        	break_state = true;
			          break;
		          }

			}
			if (break_state) break;
			
			if ( operator_reaction ("Save drawing ") ) {
				sr_ecp_msg->message("Zapisywanie trajektorii");
				mp_dtif_gen.save_file (POSE_FORCE_TORQUE_AT_FRAME);
			}
		  
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
