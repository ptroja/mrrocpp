#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "mp_t_swarm_demo_single_agent.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/maxon/dp_epos.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"
#include "ecp_mp_g_smb.h"

#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb2.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void swarmitfix::create_robots()
{
	ACTIVATE_MP_ROBOT(spkm2);
	ACTIVATE_MP_ROBOT(smb2);
}

swarmitfix::swarmitfix(lib::configurator &_config) :
		task(_config)
{

}

void swarmitfix::main_task_algorithm(void)
{

	// communication structures to send to ECP
	lib::smb::festo_command_td mp_ecp_festo_command;
	lib::epos::epos_simple_command mp_ecp_smb_epos_simple_command, mp_ecp_spkm_epos_simple_command;

	mp_ecp_smb_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;
	mp_ecp_spkm_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];

	sr_ecp_msg->message("New experimental series");

// znajdujemy sie w pozycji bazy jezdnej A
// smb - najpierw idziemy robotem w gore (wszystkie nogi w dol)
	sr_ecp_msg->message("1");

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.leg[i] = lib::smb::DOWN;
		mp_ecp_festo_command.undetachable[i] = false;
	}

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// spkm - przemieszczamy manipulator do pozycji podparcia dykty
	sr_ecp_msg->message("2");

	mp_ecp_spkm_epos_simple_command.desired_position[0] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = 0.262;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = 0;

	memcpy(mp_ecp_string, &mp_ecp_spkm_epos_simple_command, sizeof(mp_ecp_spkm_epos_simple_command));

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm2::ROBOT_NAME.c_str());

// stoimy przez dwie sekundy symulujac podparcie dykty
	sr_ecp_msg->message("3");

	wait_ms(2000);

// spkm - przemieszczamy manipulator do pozycji marszowej (opuszczamy koncowke w dol)
	sr_ecp_msg->message("4");

	mp_ecp_spkm_epos_simple_command.desired_position[0] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = 0;

	memcpy(mp_ecp_string, &mp_ecp_spkm_epos_simple_command, sizeof(mp_ecp_spkm_epos_simple_command));

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm2::ROBOT_NAME.c_str());

// smb - unosimy dwie nogi
	sr_ecp_msg->message("5");
	sr_ecp_msg->message("PODNOSZEBIE NOG za 3s");

	wait_ms(3000);

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = lib::smb::UP;
	mp_ecp_festo_command.leg[1] = lib::smb::DOWN;
	mp_ecp_festo_command.leg[2] = lib::smb::UP;

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// smb - obracamy sie wokol opuszczonej nogi do pozycji bazy jezdnej B
	sr_ecp_msg->message("6");

	mp_ecp_smb_epos_simple_command.desired_position[0] = 1;
	mp_ecp_smb_epos_simple_command.desired_position[1] = 0;

	memcpy(mp_ecp_string, &mp_ecp_smb_epos_simple_command, sizeof(mp_ecp_smb_epos_simple_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// smb - opusczamy dwie nogi, ktore byly w gorze
	sr_ecp_msg->message("7");

	sr_ecp_msg->message("Opuszczanie DWOCH NOG za 3s");

	wait_ms(3000);

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = lib::smb::DOWN;
	mp_ecp_festo_command.leg[1] = lib::smb::DOWN;
	mp_ecp_festo_command.leg[2] = lib::smb::DOWN;

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// spkm - przemieszczamy manipulator do pozycji podparcia dykty
	sr_ecp_msg->message("8");

	mp_ecp_spkm_epos_simple_command.desired_position[0] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = 0.262;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = 0;

	memcpy(mp_ecp_string, &mp_ecp_spkm_epos_simple_command, sizeof(mp_ecp_spkm_epos_simple_command));

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm2::ROBOT_NAME.c_str());

// stoimy przez dwie sekundy symulujac podparcie dykty
	sr_ecp_msg->message("9");
	wait_ms(2000);

// spkm - przemieszczamy manipulator do pozycji marszowej (opuszczamy koncowke w dol)
	sr_ecp_msg->message("10");

	mp_ecp_spkm_epos_simple_command.desired_position[0] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = 0.242;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = 0;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = 0;

	memcpy(mp_ecp_string, &mp_ecp_spkm_epos_simple_command, sizeof(mp_ecp_spkm_epos_simple_command));

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm2::ROBOT_NAME.c_str());

// smb - unosimy dwie nogi
	sr_ecp_msg->message("11");

	sr_ecp_msg->message("PODNOSZEBIE DWOCH NOG za 3s");

	wait_ms(3000);

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = lib::smb::UP;
	mp_ecp_festo_command.leg[1] = lib::smb::DOWN;
	mp_ecp_festo_command.leg[2] = lib::smb::UP;

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// smb - obracamy sie wokol opuszczonej nogi do pozycji bazy jezdnej A
	sr_ecp_msg->message("12");

	mp_ecp_smb_epos_simple_command.desired_position[0] = 0;
	mp_ecp_smb_epos_simple_command.desired_position[1] = 0;

	memcpy(mp_ecp_string, &mp_ecp_smb_epos_simple_command, sizeof(mp_ecp_smb_epos_simple_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// smb - opusczamy dwie nogi, ktore byly w gorze
	sr_ecp_msg->message("13");

	sr_ecp_msg->message("Opuszczanie NOG za 3s");

	wait_ms(3000);

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = lib::smb::DOWN;
	mp_ecp_festo_command.leg[1] = lib::smb::DOWN;
	mp_ecp_festo_command.leg[2] = lib::smb::DOWN;

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());


// smb - podnosimy wszystkie nogi
	sr_ecp_msg->message("14");

	sr_ecp_msg->message("PODNOSZEBIE WSZYSTKICH NOG za 3s");

	wait_ms(3000);

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = lib::smb::UP;
	mp_ecp_festo_command.leg[1] = lib::smb::UP;
	mp_ecp_festo_command.leg[2] = lib::smb::UP;

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());


// KONIEC
	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
