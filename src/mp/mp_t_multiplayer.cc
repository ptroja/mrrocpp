// -------------------------------------------------------------------------
//                              mp_t_multiplayer.cc
// 
// MP task for two robot multiplayer device
// 
// -------------------------------------------------------------------------


#include <stdio.h>
#include <map>

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_multiplayer.h"
#include "ecp_mp/ecp_mp_tr_player.h"

mp_task* return_created_mp_task (void)
{
	return new mp_task_multiplayer();
}

// methods fo mp template to redefine in concete class
void mp_task_multiplayer::task_initialization(void) 
{
	// Powolanie czujnikow
	transmitter_m[TRANSMITTER_PLAYER] = 
		new player_transmitter (TRANSMITTER_PLAYER, "[pcm1]", *this,
		"localhost", 6665, "position", 3, 'a');
		
	sr_ecp_msg->message("MP multiplayer task loaded");
}
 
void mp_task_multiplayer::main_task_algorithm(void)
{

	break_state = false;

	/*
  	mp_playerpos_generator mp_playerpos_gen(*this, 10); 
   	mp_h_gen.robot_m = robot_m;
   	mp_h_gen.sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
   	mp_h_gen.sensor_m[SENSOR_FORCE_POSTUMENT] = sensor_m[SENSOR_FORCE_POSTUMENT];
	*/

	// Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP multiplayer device - press start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);

	for (;;) { 

		// Zlecenie wykonania kolejnego makrokroku
		for(;;) {
			/*
			sr_ecp_msg->message("New series");
			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}

			mp_h_gen.configure(1, 0);

		    	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
			if (Move ( mp_h_gen)) {
		        	break_state = true;
		        	break;
			}
			*/
		}
	 		 	
		if (break_state) {
			break;
	
		}	
		
	        // Oczekiwanie na STOP od UI
	        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	  
	        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
	        terminate_all (robot_m);
	        break; 
	} // koniec: for(;;) - wewnetrzna petla
}
