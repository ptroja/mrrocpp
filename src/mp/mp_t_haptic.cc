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

mp_task* return_created_mp_task (void)
{
	return new mp_task_haptic();
}

// methods fo mp template to redefine in concete class
void mp_task_haptic::task_initialization(void) 
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
	
	sr_ecp_msg->message("MP haptic device loaded");
};
 
void mp_task_haptic::main_task_algorithm(void)
{

	break_state = false;

  	mp_haptic_generator mp_h_gen(*this, 10); 
   	mp_h_gen.robot_m = robot_m;
   	mp_h_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
   	mp_h_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];

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
			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}


			// mp_h_gen.sensor_m = sensor_m;
			mp_h_gen.configure(1, 0);
		    	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
			if (Move ( mp_h_gen)) {
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
};
