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
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"
#include "ecp_mp_g_smb.h"
#include "ecp_mp_g_shead.h"

namespace mrrocpp {
namespace mp {
namespace task {

#define SMB_WALK 0
#define SMB_PULL_LEGS 1

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void swarmitfix::create_robots()
{
	ACTIVATE_MP_ROBOT(spkm1);
	ACTIVATE_MP_ROBOT(smb1);
	ACTIVATE_MP_ROBOT(shead1);
}

swarmitfix::swarmitfix(lib::configurator &_config) :
		task(_config)
{

}

void swarmitfix::rotate_smb(int leg_number, double rotation)
{
	// smb - unosimy dwie nogi
	sr_ecp_msg->message("5");
	sr_ecp_msg->message("PODNOSZENIE NOG za 3s");

	wait_ms(3000);

	switch (leg_number)
	{
		case 1: {
			move_smb_legs(lib::smb::OUT, lib::smb::IN, lib::smb::IN);
		}
			break;
		case 2: {
			move_smb_legs(lib::smb::IN, lib::smb::OUT, lib::smb::IN);

		}
			break;
		case 3: {
			move_smb_legs(lib::smb::IN, lib::smb::IN, lib::smb::OUT);
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

	move_smb_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
}

void swarmitfix::move_smb_and_spkm(int leg_number, double rotation)
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
	sr_ecp_msg->message("PODNOSZENIE NOG za 3s");

	wait_ms(3000);

	switch (leg_number)
	{
		case 1: {
			move_smb_legs(lib::smb::OUT, lib::smb::IN, lib::smb::IN);
		}
			break;
		case 2: {
			move_smb_legs(lib::smb::IN, lib::smb::OUT, lib::smb::IN);

		}
			break;
		case 3: {
			move_smb_legs(lib::smb::IN, lib::smb::IN, lib::smb::OUT);
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

	move_smb_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
}


void swarmitfix::move_to_pose_and_return(
		double support_pkm_x_, double support_pkm_y_, double support_pkm_z_, double support_pkm_alpha_, double support_pkm_beta_, double support_pkm_gamma_,
		double inter_pkm_x_, double inter_pkm_y_, double inter_pkm_z_, double inter_pkm_alpha_, double inter_pkm_beta_, double inter_pkm_gamma_,
		double smb_joint_, double shead_joint_)
{
	// Move SMB and SPKM to pose.
	move_smb_external(0.0, smb_joint_);
	// Support interpose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, inter_pkm_x_, inter_pkm_y_, inter_pkm_z_, inter_pkm_alpha_, inter_pkm_beta_, inter_pkm_gamma_);
	// Rotate shead.
	move_shead_joints(shead_joint_);
	// Support.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, support_pkm_x_, support_pkm_y_, support_pkm_z_, support_pkm_alpha_, support_pkm_beta_, support_pkm_gamma_);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	// Support interpose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, inter_pkm_x_, inter_pkm_y_, inter_pkm_z_, inter_pkm_alpha_, inter_pkm_beta_, inter_pkm_gamma_);
	// Neutral.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, -0.04, 0.4, 0, -0.92, 0);

}

void swarmitfix::main_task_algorithm(void)
{
	/*
	    neutral pose OK
	    tool:  -0.1412 -0.04 0.5718 3.1416 0.137 0
	    wrist: 0.15 -0.04 0.4 0 -0.92 0
	*/
	// Move to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, -0.04, 0.4, 0, -0.92, 0);
	// Move shead to synchro position.
	move_shead_joints(0.0);


#if(SMB_PULL_LEGS)
	// Pull out all SMB legs.
	move_smb_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
#endif
/*
    Podparcie nr 1: smb rot = 1
    tool:  -0.33 0.0012 0.625 -0.8193 0.029 -2.3437
    wrist: -0.0693 0 0.4097 0 -0.763 -0.03

    odejście:
    tool:  -0.33 0 0.59 3.1416 0.02 0
    wrist: -0.0609 0 0.3853 0 -0.803 0
*/
	move_to_pose_and_return(-0.0693, 0, 0.4097, 0, -0.763, -0.03, -0.0609, 0, 0.3853, 0, -0.803, 0, 1, 0.5);
	/*
	    Podparcie 2: smb rot = 0, legs out
	    tool:   -0.2919 0 0.626 -3.1416 -0.03 0
	    wrist:  -0.0333 0 0.4081 0 -0.753 0

	    odejście z podparcia nr 1:
	    tool:  -0.2919 0 0.6 -3.1416 0 0
	    wrist: -0.0269 0 0.39 0 -0.783 0
	*/
	move_to_pose_and_return(-0.0333, 0, 0.4081, 0, -0.753, 0, -0.0269, 0, 0.39, 0, -0.783, 0, 0, -0.5);
	/*
	    Podparcie nr 3: smb rot = -1
	    tool: -0.3691 0 0.627 3.1416 -0.05 0
	    wrist: -0.1149 0 0.404 0 -0.733 0

	    odejście:
	    tool:  -0.3691 0 0.5847 3.1416 0.02 0
	    wrist: -0.1 0 0.38 0 -0.803 0
	*/
	move_to_pose_and_return(-0.1149, 0, 0.404, 0, -0.733, 0, -0.1, 0, 0.38, 0, -0.803, 0, -1, 0);


#if(SMB_WALK)
	// Move to SMB position 2 - rotate around leg 3 by 60 degrees.
	rotate_smb(3, 1);

	// Move SMB and SPKM to pose 4.
	move_smb_external(0, 2.0);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.05, 0, 0.425, 0, -0.81, -0.1);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.05, 0, 0.445, 0, -0.81, -0.1);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.05, 0, 0.425, 0, -0.81, -0.1);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, 0, 0.405, 0, -1.045, 0);

	// Move SMB and SPKM to pose 3.
	move_smb_external(0, -1.500);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.409, 0, -0.868, 0.09);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.429, 0, -0.868, 0.09);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.419, 0, -0.868, 0.09);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, 0, 0.405, 0, -1.045, 0);

	// Move SMB and SPKM to pose 5.
	move_smb_external(0, -3.0);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.404, 0, -0.83, -0.02);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.424, 0, -0.83, -0.02);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.404, 0, -0.83, -0.02);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, 0, 0.405, 0, -1.045, 0);

	// Move to SMB position 1 - rotate around leg 3 by -60 degrees.
	rotate_smb(3, -1);
	// Move to start position.
	move_smb_external(0, 0);
#endif

	// Rotate SMB to synchro position.
	move_smb_external(0.0, 0.0);
	wait_ms(1000);

#if(SMB_PULL_LEGS)
	move_smb_legs(lib::smb::IN, lib::smb::IN, lib::smb::IN);
#endif
// KONIEC
//	sr_ecp_msg->message("END");

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

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb1::ROBOT_NAME.c_str());

}

void swarmitfix::move_smb_external(double legs_rotation_, double pkm_rotation_)
{
	lib::smb::smb_epos_simple_command mp_ecp_smb_epos_simple_command;
	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];
	mp_ecp_smb_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	mp_ecp_smb_epos_simple_command.base_vs_bench_rotation = legs_rotation_;
	mp_ecp_smb_epos_simple_command.pkm_vs_base_rotation = pkm_rotation_;

	memcpy(mp_ecp_string, &mp_ecp_smb_epos_simple_command, sizeof(mp_ecp_smb_epos_simple_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb1::ROBOT_NAME.c_str());

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

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm1::ROBOT_NAME.c_str());

}

void swarmitfix::move_shead_joints(double x1)
{
	lib::epos::epos_simple_command mp_ecp_shead_epos_simple_command;
	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];
	mp_ecp_shead_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	mp_ecp_shead_epos_simple_command.desired_position[0] = x1;

	memcpy(mp_ecp_string, &mp_ecp_shead_epos_simple_command, sizeof(mp_ecp_shead_epos_simple_command));

	set_next_ecp_state(ecp_mp::shead::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::shead1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::shead1::ROBOT_NAME.c_str());

}

void swarmitfix::move_spkm_external(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double x1, double x2, double x3, double x4, double x5, double x6)
{
	lib::spkm::spkm_epos_simple_command mp_ecp_spkm_epos_simple_command;
	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];
	mp_ecp_spkm_epos_simple_command.motion_variant = motion_variant_;
	mp_ecp_spkm_epos_simple_command.pose_specification = lib::spkm::XYZ_EULER_ZYZ;
	mp_ecp_spkm_epos_simple_command.estimated_time = 1.2;

	mp_ecp_spkm_epos_simple_command.desired_position[0] = x1;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = x2;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = x3;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = x4;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = x5;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = x6;

	memcpy(mp_ecp_string, &mp_ecp_spkm_epos_simple_command, sizeof(mp_ecp_spkm_epos_simple_command));

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::spkm1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm1::ROBOT_NAME.c_str());

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
