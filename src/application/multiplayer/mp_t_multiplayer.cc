// -------------------------------------------------------------------------
//                              task/mp_t_multiplayer.cc
//
// MP task for two robot multiplayer device
//
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <map>
#include <string.h>

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp_t_multiplayer.h"
#include "ecp_mp/task/ecp_mp_t_festival.h"
#include "ecp_mp/task/ecp_mp_t_player.h"
#include "ecp_mp_t_multiplayer.h"
#include "ecp/festival/generator/ecp_g_festival.h"

namespace mrrocpp {
namespace mp {
namespace task {

void multiplayer::move_electron_robot(const lib::playerpos_goal_t &goal)
{
	set_next_playerpos_goal (lib::ROBOT_ELECTRON, goal);
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_ELECTRON, lib::ROBOT_ELECTRON);
}

multiplayer::multiplayer(lib::configurator &_config) : task(_config)
{
}

task* return_created_mp_task (lib::configurator &_config)
{
	return new multiplayer(_config);
}

#define FRICTION_CORRECTOR	1.5

void multiplayer::main_task_algorithm(void)
{
  	//mp_playerpos_generator playerpos_gen(*this);
   	//playerpos_gen.transmitter_m = this->transmitter_m;
  	//mp_playerspeech_generator playerspeech_gen(*this);
   	//playerspeech_gen.transmitter_m = this->transmitter_m;

	sr_ecp_msg->message("Nowy makrokrok");

	// pozycja robota mobilnego
	lib::playerpos_goal_t goal;
#if 0
	// dojezdzanie
	goal.forward(1.2); move_electron_robot(goal);
	goal.turn(-M_PI_2); move_electron_robot(goal);
	goal.forward(.6); move_electron_robot(goal);

	// powrot
	goal.turn(-M_PI_2); move_electron_robot(goal);
	goal.turn(-M_PI_2*0.8); move_electron_robot(goal);

	goal.forward(0.85); move_electron_robot(goal);
	goal.turn(M_PI_2*0.96); move_electron_robot(goal);
	goal.forward(1.0); move_electron_robot(goal);
#endif
#if 1
	// USTAWIENIE POCZATKOWE
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_init.trj", 0, 1, lib::ROBOT_IRP6OT_M);
	set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "inicjalizuje~ zadanie", 0, 1, lib::ROBOT_FESTIVAL);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			2, 1,
			lib::ROBOT_IRP6OT_M, lib::ROBOT_FESTIVAL,
			lib::ROBOT_IRP6OT_M
			);

	set_next_ecps_state( (int) ecp_mp::task::MULTIPLAYER_GRIPPER_OPENING, 0, NULL, 0, 1, lib::ROBOT_IRP6OT_M);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			2, 2,
			lib::ROBOT_IRP6OT_M, lib::ROBOT_FESTIVAL,
			lib::ROBOT_IRP6OT_M, lib::ROBOT_FESTIVAL
			);
#endif
#if 1
	// OCZEKIWANIE NA POLECENIE (komuikat z festivala)
	set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "oczekuje~ na polecenie", 0, 1, lib::ROBOT_FESTIVAL);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		(1, 1, lib::ROBOT_FESTIVAL, lib::ROBOT_FESTIVAL);

	bool komenda_rozpoznana = false;
	do
	{
		// OCZEKIWANIE NA POLECENIE (faktyczne oczekiwanie)
		set_next_ecps_state (ecp_mp::task::ECP_GEN_SPEECHRECOGNITION, 0, NULL, 0, 1, lib::ROBOT_SPEECHRECOGNITION);
		// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
				(1, 1, lib::ROBOT_SPEECHRECOGNITION, lib::ROBOT_SPEECHRECOGNITION);

		char *qq = robot_m[lib::ROBOT_SPEECHRECOGNITION]->ecp_reply_package.commandRecognized;
		printf("commandRecognized = \"%s\"\n", qq);

		const char *komunikat;

		if (!strcmp(qq, "PODAJ_KOSTKE")) {
			komenda_rozpoznana = true;
			komunikat = "polecenie rozpoznane";
		} else {
			komunikat = "polecenie nie rozpoznane";
		}

		// komuikat z festivala
		set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, komunikat, 0, 1, lib::ROBOT_FESTIVAL);
		// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			(1, 1, lib::ROBOT_FESTIVAL, lib::ROBOT_FESTIVAL);
	} while (komenda_rozpoznana == false);

	// FAZA DOJEZDZANIA DO POZYCJI PODNOSZENIA KOSTKI
	goal.forward(1.2);

	set_next_playerpos_goal (lib::ROBOT_ELECTRON, goal);
	set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "jade~ przekazac~ kostke~", 0, 1, lib::ROBOT_FESTIVAL);
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_grab.trj", 0, 1, lib::ROBOT_IRP6OT_M);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			(3, 3,
					lib::ROBOT_ELECTRON, lib::ROBOT_FESTIVAL, lib::ROBOT_IRP6OT_M,
					lib::ROBOT_ELECTRON, lib::ROBOT_FESTIVAL, lib::ROBOT_IRP6OT_M
			);

	goal.turn(-M_PI_2);

	set_next_playerpos_goal (lib::ROBOT_ELECTRON, goal);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		(1, 1, lib::ROBOT_ELECTRON, lib::ROBOT_ELECTRON);

	goal.forward(0.6);

	set_next_playerpos_goal (lib::ROBOT_ELECTRON, goal);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		(1, 1, lib::ROBOT_ELECTRON, lib::ROBOT_ELECTRON);

	// FAZA PRZECHWYTYWANIA KOSTKI
	set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "drugi robot podniesie kostke~", 0, 1, lib::ROBOT_FESTIVAL);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		(1, 1, lib::ROBOT_FESTIVAL, lib::ROBOT_FESTIVAL);

	//biasowanie czujnika sily
	set_next_ecps_state(ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE, 0, "", 0, 1, lib::ROBOT_IRP6OT_M);

	//oczekiwanie na ustalenie balansu bieli w kamerze
	wait_ms(7000);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);

	//podjazd do chwytu obiektu przez serwowizje
	set_next_ecps_state(ecp_mp::task::ECP_GEN_TAKE_FROM_ROVER, 0, "", 0, 1, lib::ROBOT_IRP6OT_M);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);

	//chwycenie
	set_next_ecps_state(ecp_mp::task::ECP_GEN_GRAB_FROM_ROVER, 0, "", 0, 1, lib::ROBOT_IRP6OT_M);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);

	//RUCH DO GORY
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_up.trj", 0, 1, lib::ROBOT_IRP6OT_M);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots( 1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);

	//DOJEZDZANIE DO POZYCJI PRZEKAZANIA KOSTKI
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_pass.trj", 0, 1, lib::ROBOT_IRP6OT_M);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots( 1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);

	//ROZWARCIE SZCZEK
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_wide.trj", 0, 1, lib::ROBOT_IRP6OT_M);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots( 1, 1, lib::ROBOT_IRP6OT_M, lib::ROBOT_IRP6OT_M);

	// FAZA ODBIERANIA KOSTKI
	set_next_ecps_state( (int) ecp_mp::task::ECP_WEIGHT_MEASURE_GENERATOR, 0, NULL, 0, 1, lib::ROBOT_IRP6OT_M);
	set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "prosze~ odbierz kostke~", 0, 1, lib::ROBOT_FESTIVAL);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			2, 2,
			lib::ROBOT_IRP6OT_M, lib::ROBOT_FESTIVAL,
			lib::ROBOT_IRP6OT_M, lib::ROBOT_FESTIVAL
			);

	// FAZA POWROTU DO USTAWIENIA POCZATKOWEGO

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/multiplayer/irp6ot_sm_init.trj", 0, 1, lib::ROBOT_IRP6OT_M);
	set_next_ecps_state (ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "zadanie wykonane", 0, 1, lib::ROBOT_FESTIVAL);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			2, 2,
			lib::ROBOT_FESTIVAL, lib::ROBOT_IRP6OT_M,
			lib::ROBOT_FESTIVAL, lib::ROBOT_IRP6OT_M
			);

	// powrot
	goal.turn(-M_PI_2); move_electron_robot(goal);
	goal.turn(-M_PI_2*0.8); move_electron_robot(goal);

	goal.forward(0.84); move_electron_robot(goal);
	goal.turn(M_PI_2*0.96); move_electron_robot(goal);
	goal.forward(0.96); move_electron_robot(goal);
#endif

}


} // namespace task
} // namespace mp
} // namespace mrrocpp
