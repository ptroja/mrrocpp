#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"

#include "mp_t_swarm_demo_single_agent.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/maxon/dp_epos.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"
#include "ecp_mp_g_smb.h"

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

void swarmitfix::move_smb(int leg_number, double rotation)
{
	// spkm - przemieszczamy manipulator do pozycji podparcia dykty
	sr_ecp_msg->message("2");

	move_spkm_joints(0.28, 0.292, 0.28, 0, 0, 0);

	// stoimy przez dwie sekundy symulujac podparcie dykty
	sr_ecp_msg->message("3");

	wait_ms(2000);

	// spkm - przemieszczamy manipulator do pozycji marszowej (opuszczamy koncowke w dol)
	sr_ecp_msg->message("4");

	move_spkm_joints(0.242, 0.242, 0.242, 0, 0, 0);

	// smb - unosimy dwie nogi
	sr_ecp_msg->message("5");
	sr_ecp_msg->message("PODNOSZEBIE NOG za 3s");

	wait_ms(3000);

	switch (leg_number)
	{
		case 1: {
			move_smb_legs(lib::smb::DOWN, lib::smb::UP, lib::smb::UP);
		}
			break;
		case 2: {
			move_smb_legs(lib::smb::UP, lib::smb::DOWN, lib::smb::UP);

		}
			break;
		case 3: {
			move_smb_legs(lib::smb::UP, lib::smb::UP, lib::smb::DOWN);
		}
			break;
		default:
			break;
	}

	// smb - obracamy sie wokol opuszczonej nogi do pozycji bazy jezdnej B
	sr_ecp_msg->message("6");

	move_smb_external(rotation, 0);

	// smb - opusczamy dwie nogi, ktore byly w gorze
	sr_ecp_msg->message("7");

	sr_ecp_msg->message("Opuszczanie DWOCH NOG za 3s");

	wait_ms(3000);

	move_smb_legs(lib::smb::DOWN, lib::smb::DOWN, lib::smb::DOWN);
}

void swarmitfix::main_task_algorithm(void)
{

	sr_ecp_msg->message("New experimental series");

// znajdujemy sie w pozycji bazy jezdnej A
// smb - najpierw idziemy robotem w gore (wszystkie nogi w dol)
	sr_ecp_msg->message("1");

	move_smb_legs(lib::smb::DOWN, lib::smb::DOWN, lib::smb::DOWN);
	move_smb(2, -1);
	move_smb(1, -1);
	move_smb(3, -1);
	move_smb(2, -1);
	move_smb(1, -1);
	move_smb(3, 5);

// smb - podnosimy wszystkie nogi
	sr_ecp_msg->message("14");

	sr_ecp_msg->message("PODNOSZEBIE WSZYSTKICH NOG za 1s");

	wait_ms(1000);

	move_smb_legs(lib::smb::UP, lib::smb::UP, lib::smb::UP);

// KONIEC
	sr_ecp_msg->message("END");

}

void swarmitfix::move_smb_legs(lib::smb::FESTO_LEG l1, lib::smb::FESTO_LEG l2, lib::smb::FESTO_LEG l3)
{
	lib::smb::festo_command_td mp_ecp_festo_command;
	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = l1;
	mp_ecp_festo_command.leg[1] = l2;
	mp_ecp_festo_command.leg[2] = l3;

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

}

void swarmitfix::move_smb_external(double x1, double x2)
{
	lib::epos::epos_simple_command mp_ecp_smb_epos_simple_command;
	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];
	mp_ecp_smb_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	mp_ecp_smb_epos_simple_command.desired_position[0] = x1;
	mp_ecp_smb_epos_simple_command.desired_position[1] = x2;

	memcpy(mp_ecp_string, &mp_ecp_smb_epos_simple_command, sizeof(mp_ecp_smb_epos_simple_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

}

void swarmitfix::move_spkm_joints(double x1, double x2, double x3, double x4, double x5, double x6)
{
	lib::epos::epos_simple_command mp_ecp_spkm_epos_simple_command;
	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];
	mp_ecp_spkm_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	mp_ecp_spkm_epos_simple_command.desired_position[0] = x1;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = x2;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = x3;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = x4;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = x5;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = x6;

	memcpy(mp_ecp_string, &mp_ecp_spkm_epos_simple_command, sizeof(mp_ecp_spkm_epos_simple_command));

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm2::ROBOT_NAME.c_str());

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
