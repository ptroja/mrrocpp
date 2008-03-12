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
#include "ecp_mp/ecp_mp_t_festival.h"
#include "ecp_mp/ecp_mp_t_player.h"
#include "ecp_mp/ecp_mp_t_multiplayer.h"
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
#if 0
			if (set_next_ecps_state (ECP_GEN_SPEECHRECOGNITION, 0, NULL, 1, ROBOT_SPEECHRECOGNITION)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_SPEECHRECOGNITION, ROBOT_SPEECHRECOGNITION)) {
				break_state = true;
		       	break;
			}

			char *qq = robot_m[ROBOT_SPEECHRECOGNITION]->ecp_td.commandRecognized;
			printf("commandRecognized = \"%s\"\n", qq);
			
			if (set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "raz dwa trzy cztery pie~c~ szes~c~ siedem osiem", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
#endif
	
			// pozycja robota mobilnego
			playerpos_goal_t goal;
			
			goal.setGoal(1.0, 0.0, 0.0);

			if (set_next_playerpos_goal (ROBOT_ELECTRON, goal)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_ELECTRON, ROBOT_ELECTRON)) {
				break_state = true;
		       	break;
			}
			
			goal.turn(-1.5*M_PI_2);

			if (set_next_playerpos_goal (ROBOT_ELECTRON, goal)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_ELECTRON, ROBOT_ELECTRON)) {
				break_state = true;
		       	break;
			}

			goal.forward(1.0);

			if (set_next_playerpos_goal (ROBOT_ELECTRON, goal)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_ELECTRON, ROBOT_ELECTRON)) {
				break_state = true;
		       	break;
			}			
#if 0
	
			// USTAWIENIE POCZATKOWE
			if (set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_init.trj", 1, ROBOT_IRP6_ON_TRACK) ||
				set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "inicjalizuje~ zadanie", 1, ROBOT_FESTIVAL)) {
				break_state = true;
				break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
					2, 1,
					ROBOT_IRP6_ON_TRACK, ROBOT_FESTIVAL,
					ROBOT_IRP6_ON_TRACK
					)) {
				break_state = true;
				break;
			}
			
			if (set_next_ecps_state( (int) MULTIPLAYER_GRIPPER_OPENING, 0, NULL, 1, ROBOT_IRP6_ON_TRACK) ) {
				break_state = true;
				break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
					2, 2,
					ROBOT_IRP6_ON_TRACK, ROBOT_FESTIVAL,
					ROBOT_IRP6_ON_TRACK, ROBOT_FESTIVAL
					)) {
				break_state = true;
				break;
			}
			
			// OCZEKIWANIE NA POLECENIE
			if (set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "oczekuje~ na polecenie", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}

			// FAZA DOJEZDZANIA DO POZYCJI PODNOSZENIA KOSTKI
			goal.setGoal(1.0, 0.0, 0.0);
			
			if (set_next_playerpos_goal (ROBOT_ELECTRON, goal) ||
				set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "jade~ przekazac~ kostke~", 1, ROBOT_FESTIVAL) ||
				set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_grab.trj", 1, ROBOT_IRP6_ON_TRACK)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
					(3, 3,
							ROBOT_ELECTRON, ROBOT_FESTIVAL, ROBOT_IRP6_ON_TRACK,
							ROBOT_ELECTRON, ROBOT_FESTIVAL, ROBOT_IRP6_ON_TRACK
					)) {
				break_state = true;
				break;
			}

			goal.turn(-1.5*M_PI_2);

			if (set_next_playerpos_goal (ROBOT_ELECTRON, goal)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_ELECTRON, ROBOT_ELECTRON)) {
				break_state = true;
		       	break;
			}

			goal.forward(1.0);

			if (set_next_playerpos_goal (ROBOT_ELECTRON, goal)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_ELECTRON, ROBOT_ELECTRON)) {
				break_state = true;
		       	break;
			}			

			// FAZA PRZECHWYTYWANIA KOSTKI
			if (set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "teraz drugi robot podniesie kostke~!", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}

			// TODO: tutuaj ma byc uruchomienie generatora Siodmego...
			if (set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "udajemy z*e drugi robot podnio~sl/ kostke", 1, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		        (1, 1, ROBOT_FESTIVAL, ROBOT_FESTIVAL)) {
				break_state = true;
		       	break;
			}

			// DOJEZDZANIE DO POZYCJI PRZEKAZANIA KOSTKI
			if (set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_pass.trj", 1, ROBOT_IRP6_ON_TRACK) ||
				set_next_ecps_state (ECP_GEN_FESTIVAL, festival_generator::POLISH_VOICE, "podaje kostke~", 1, ROBOT_FESTIVAL)) {
				break_state = true;
				break;
			}
			// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
			if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
					2, 2,
					ROBOT_IRP6_ON_TRACK, ROBOT_FESTIVAL,
					ROBOT_IRP6_ON_TRACK, ROBOT_FESTIVAL
					)) {
				break_state = true;
				break;
			}			
#endif
		} while(0);
		
		if (break_state)
			break;

        // Oczekiwanie na STOP od UI
        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
	}
}
