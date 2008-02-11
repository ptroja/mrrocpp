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
#include "mp/mp_g_playerpos.h"
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
		"192.168.18.30", 6665, "position", 3, 'a');
	
	printf("mp_task_multiplayer.transmitter_m.count() = %d @ %s:%d\n", transmitter_m.size(), __FILE__, __LINE__);
		
	sr_ecp_msg->message("MP multiplayer task loaded");
}
 
void mp_task_multiplayer::main_task_algorithm(void)
{
  	mp_playerpos_generator playerpos_gen(*this, 1.0); 
   	playerpos_gen.transmitter_m = this->transmitter_m;
   	
   	printf("this->transmitter_m.count() = %d @ %s:%d\n", this->transmitter_m.size(), __FILE__, __LINE__);
   	printf("playerpos_gen.transmitter_m.count() = %d @ %s:%d\n", playerpos_gen.transmitter_m.size(), __FILE__, __LINE__);

	// Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP multiplayer device - press start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);
	
	bool break_state = false;

	for (;;) { 

		// Zlecenie wykonania kolejnego makrokroku
		for (;;) {
			sr_ecp_msg->message("Nowy makrokrok");

			if (Move ( playerpos_gen)) {
		        	break_state = true;
		        	break;
			}
		}
		
		if (break_state)
			break;

        // Oczekiwanie na STOP od UI
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
	}
}
