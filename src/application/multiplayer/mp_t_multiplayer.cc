// -------------------------------------------------------------------------
//                              task/mp_t_multiplayer.cc
//
// MP task for two robot multiplayer device
//
// -------------------------------------------------------------------------

#include <cstdio>
#include <cmath>
#include <map>
#include <cstring>

#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_generator.h"
#include "base/mp/mp_robot.h"
#include "base/mp/MP_main_error.h"
#include "mp_t_multiplayer.h"
#include "robot/festival/ecp_mp_t_festival.h"
#include "robot/player/ecp_mp_t_player.h"
#include "ecp_mp_t_multiplayer.h"
#include "robot/festival/ecp_g_festival.h"
#include "robot/festival/const_festival.h"
#include "robot/player/const_player.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/force/ecp_mp_g_weight_measure.h"
#include "subtask/ecp_mp_st_gripper_opening.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6m/mp_r_irp6m.h"
#include "robot/speaker/mp_r_speaker.h"
#include "robot/polycrank/mp_r_polycrank.h"
#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/shead/mp_r_shead.h"
#include "robot/spkm/mp_r_spkm.h"
#include "robot/smb/mp_r_smb.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"
#include "robot/player/const_player.h"

namespace mrrocpp {
namespace mp {
namespace task {

void multiplayer::move_electron_robot(const lib::playerpos_goal_t &goal)
{
	set_next_playerpos_goal(lib::electron::ROBOT_NAME.c_str(), goal);
	run_extended_empty_gen_and_wait(1, 1, lib::electron::ROBOT_NAME.c_str(), lib::electron::ROBOT_NAME.c_str());
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void multiplayer::create_robots()
{
	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(speaker);
	ACTIVATE_MP_ROBOT(irp6m);
	ACTIVATE_MP_ROBOT(polycrank);
	ACTIVATE_MP_ROBOT(bird_hand);
	ACTIVATE_MP_ROBOT(spkm);
	ACTIVATE_MP_ROBOT(smb);
	ACTIVATE_MP_ROBOT(shead);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);

	ACTIVATE_MP_DEFAULT_ROBOT(electron);
	ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

multiplayer::multiplayer(lib::configurator &_config) :
	task(_config)
{
}

task* return_created_mp_task(lib::configurator &_config)
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
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, 0, "trj/multiplayer/irp6ot_sm_init.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "inicjalizuje~ zadanie", 0, 1, lib::festival::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(2, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	set_next_ecps_state(ecp_mp::sub_task::ECP_ST_GRIPPER_OPENING, 0, NULL, 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(2, 2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str());
#endif
#if 1
	// OCZEKIWANIE NA POLECENIE (komuikat z festivala)
	set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "oczekuje~ na polecenie", 0, 1, lib::festival::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::festival::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str());

	bool komenda_rozpoznana = false;
	do {
		// OCZEKIWANIE NA POLECENIE (faktyczne oczekiwanie)
		set_next_ecps_state(ecp_mp::task::ECP_GEN_SPEECHRECOGNITION, 0, NULL, 0, 1, lib::speechrecognition::ROBOT_NAME.c_str());
		// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
		run_extended_empty_gen_and_wait(1, 1, lib::speechrecognition::ROBOT_NAME.c_str(), lib::speechrecognition::ROBOT_NAME.c_str());

		char *qq = robot_m[lib::speechrecognition::ROBOT_NAME]->ecp_reply_package.recognized_command;
		printf("commandRecognized = \"%s\"\n", qq);

		const char *komunikat;

		if (!strcmp(qq, "PODAJ_KOSTKE")) {
			komenda_rozpoznana = true;
			komunikat = "polecenie rozpoznane";
		} else {
			komunikat = "polecenie nie rozpoznane";
		}

		// komuikat z festivala
		set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, komunikat, 0, 1, lib::festival::ROBOT_NAME.c_str());
		// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
		run_extended_empty_gen_and_wait(1, 1, lib::festival::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str());
	} while (komenda_rozpoznana == false);

	// FAZA DOJEZDZANIA DO POZYCJI PODNOSZENIA KOSTKI
	goal.forward(1.2);

	set_next_playerpos_goal(lib::electron::ROBOT_NAME.c_str(), goal);
	set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "jade~ przekazac~ kostke~", 0, 1, lib::festival::ROBOT_NAME.c_str());
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, 0, "trj/multiplayer/irp6ot_sm_grab.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(3, 3, lib::electron::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::electron::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	goal.turn(-M_PI_2);

	set_next_playerpos_goal(lib::electron::ROBOT_NAME.c_str(), goal);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::electron::ROBOT_NAME.c_str(), lib::electron::ROBOT_NAME.c_str());

	goal.forward(0.6);

	set_next_playerpos_goal(lib::electron::ROBOT_NAME.c_str(), goal);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::electron::ROBOT_NAME.c_str(), lib::electron::ROBOT_NAME.c_str());

	// FAZA PRZECHWYTYWANIA KOSTKI
	set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "drugi robot podniesie kostke~", 0, 1, lib::festival::ROBOT_NAME.c_str());

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::festival::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str());

	//biasowanie czujnika sily
	set_next_ecps_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, 0, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());

	//oczekiwanie na ustalenie balansu bieli w kamerze
	wait_ms(7000);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	//podjazd do chwytu obiektu przez serwowizje
	set_next_ecps_state(ecp_mp::task::ECP_GEN_TAKE_FROM_ROVER, 0, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	//chwycenie
	set_next_ecps_state(ecp_mp::task::ECP_GEN_GRAB_FROM_ROVER, 0, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	//RUCH DO GORY
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, 0, "trj/multiplayer/irp6ot_sm_up.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	//DOJEZDZANIE DO POZYCJI PRZEKAZANIA KOSTKI
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, 0, "trj/multiplayer/irp6ot_sm_pass.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	//ROZWARCIE SZCZEK
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, 0, "trj/multiplayer/irp6ot_sm_wide.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	// FAZA ODBIERANIA KOSTKI
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE, 0, NULL, 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "prosze~ odbierz kostke~", 0, 1, lib::festival::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(2, 2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str());

	// FAZA POWROTU DO USTAWIENIA POCZATKOWEGO

	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, 0, "trj/multiplayer/irp6ot_sm_init.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "zadanie wykonane", 0, 1, lib::festival::ROBOT_NAME.c_str());
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie generatorow ECP
	run_extended_empty_gen_and_wait(2, 2, lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	// powrot
	goal.turn(-M_PI_2);
	move_electron_robot(goal);
	goal.turn(-M_PI_2 * 0.8);
	move_electron_robot(goal);

	goal.forward(0.84);
	move_electron_robot(goal);
	goal.turn(M_PI_2 * 0.96);
	move_electron_robot(goal);
	goal.forward(0.96);
	move_electron_robot(goal);
#endif

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
