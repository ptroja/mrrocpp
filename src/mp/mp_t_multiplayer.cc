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
#include "ecp_mp/ecp_mp_t_player.h"
#include "ecp/festival/ecp_g_festival.h"

mp_task_multiplayer::mp_task_multiplayer(configurator &_config) : mp_task(_config)
{
}

mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_multiplayer(_config);
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
			if (set_next_ecps_state (ECP_GEN_SPEECHRECOGNITION, 0, NULL, 1, ROBOT_SPEECHRECOGNITION)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_SPEECHRECOGNITION, ROBOT_SPEECHRECOGNITION)) {
				break_state = true;
		       	break;
			}
			*/
			
			/*
			char *qq = robot_m[ROBOT_SPEECHRECOGNITION]->ecp_td.commandRecognized;
			printf("commandRecognized = \"%s\"\n", qq);
			*/
			
			if (set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "raz dwa trzy cztery pie~c~ szes~c~ siedem osiem", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}		
		} while(0);
		
		if (break_state)
			break;

        // Oczekiwanie na STOP od UI
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
	}
}