// -------------------------------------------------------------------------
//                              mp_t_multiplayer.cc
// 
// MP task for two robot multiplayer device
// 
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <map>

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_multiplayer.h"

/*
#include "mp/mp_g_playerpos.h"
#include "mp/mp_g_playerspeech.h"
#include "ecp_mp/ecp_mp_tr_player.h"
*/
#include "ecp_mp/ecp_mp_t_festival.h"
#include "ecp/festival/ecp_g_festival.h"

mp_task* return_created_mp_task (void)
{
	return new mp_task_multiplayer();
}

void mp_task_multiplayer::task_initialization(void) 
{
	sr_ecp_msg->message("MP multiplayer task loaded");
}
 
void mp_task_multiplayer::main_task_algorithm(void)
{
  	//mp_playerpos_generator playerpos_gen(*this); 
   	//playerpos_gen.transmitter_m = this->transmitter_m;
  	//mp_playerspeech_generator playerspeech_gen(*this); 
   	//playerspeech_gen.transmitter_m = this->transmitter_m;
   	
	bool break_state = false;

	for (;;) { 
	// Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP multiplayer device - press start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);
	
		do {
			sr_ecp_msg->message("Nowy makrokrok");
			/*
			playerpos_gen.set_target(1.0, 0.0, 0.0);
			if (Move(playerpos_gen)) {
		        	break_state = true;
		        	break;
			}
			
			playerpos_gen.set_target(1.0, 0.0, M_PI_2);			
			if (Move(playerpos_gen)) {
		        	break_state = true;
		        	break;
			}

			playerpos_gen.set_target(1.0, 0.5, M_PI_2);
			if (Move(playerpos_gen)) {
		        	break_state = true;
		        	break;
			}
			*/		

			if (set_next_ecps_state ((int) ECP_GEN_FESTIVAL, festival_generator::ENGLISH_VOICE, "witam serdecznie!", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			
			if (set_next_ecps_state ((int) ECP_GEN_FESTIVAL, 0, "dobry wieczo~r pan~stwu", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}			
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}			

			/*
			playerspeech_gen.set_phrase("robot programming framework");
			if (Move(playerspeech_gen)) {
		        	break_state = true;
		        	break;
			}
			*/
			
		} while(0);
		
		if (break_state)
			break;

        // Oczekiwanie na STOP od UI
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
	}
}