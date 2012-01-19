#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"

#include "mp_t_swarmitfix_demo_base.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"
#include "ecp_mp_g_smb.h"
#include "ecp_mp_g_shead.h"

namespace mrrocpp {
namespace mp {
namespace task {

swarmitfix_demo_base::swarmitfix_demo_base(lib::configurator &config_) :
		task(config_)
{

}

void swarmitfix_demo_base::rotate_smb(int leg_number_, int rotation_)
{
	sr_ecp_msg->message("swarmitfix_demo_base::rotate_smb");

	// Pull all legs in except the one around which the rotation_ will be performed.
	switch (leg_number_)
	{
		case 1:
			sr_ecp_msg->message("swarmitfix_demo_base::rotate_smb: OUT, IN, IN");
			move_smb_legs(lib::smb::OUT, lib::smb::IN, lib::smb::IN);
			break;
		case 2:
			sr_ecp_msg->message("swarmitfix_demo_base::rotate_smb: IN, OUT, IN");
			move_smb_legs(lib::smb::IN, lib::smb::OUT, lib::smb::IN);
			break;
		case 3:
			sr_ecp_msg->message("swarmitfix_demo_base::rotate_smb: IN, IN, OUT");
			move_smb_legs(lib::smb::IN, lib::smb::IN, lib::smb::OUT);
			break;
		default:
			break;
	}

	// Rotate around the leg - the SPKM rotation is set to zero.
	move_smb_external(rotation_, 0);

	// Pull all legs out.
	sr_ecp_msg->message("swarmitfix_demo_base::rotate_smb: OUT, OUT, OUT");
	move_smb_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
}


void swarmitfix_demo_base::move_to_pose_and_return(double support_pkm_x_, double support_pkm_y_, double support_pkm_z_, double support_pkm_alpha_, double support_pkm_beta_, double support_pkm_gamma_, double inter_pkm_x_, double inter_pkm_y_, double inter_pkm_z_, double inter_pkm_alpha_, double inter_pkm_beta_, double inter_pkm_gamma_, double smb_joint_, double shead_joint_)
{
	// Move SMB and SPKM to pose.
	move_smb_external(0.0, smb_joint_);
	// Support interpose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, inter_pkm_x_, inter_pkm_y_, inter_pkm_z_, inter_pkm_alpha_, inter_pkm_beta_, inter_pkm_gamma_);
	// Rotate shead.
	move_shead_joints(shead_joint_);
	// Support.
	move_spkm_external(lib::epos::OPERATIONAL, support_pkm_x_, support_pkm_y_, support_pkm_z_, support_pkm_alpha_, support_pkm_beta_, support_pkm_gamma_);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	// Support interpose.
	move_spkm_external(lib::epos::OPERATIONAL, inter_pkm_x_, inter_pkm_y_, inter_pkm_z_, inter_pkm_alpha_, inter_pkm_beta_, inter_pkm_gamma_);
	// Neutral.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, -0.04, 0.4, 0, -0.92, 0);

}

void swarmitfix_demo_base::move_smb_legs(lib::smb::FESTO_LEG l1_, lib::smb::FESTO_LEG l2_, lib::smb::FESTO_LEG l3_)
{
	sr_ecp_msg->message("swarmitfix_demo_base::move_smb_legs");
	lib::smb::festo_command_td mp_ecp_festo_command;

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = l1_;
	mp_ecp_festo_command.leg[1] = l2_;
	mp_ecp_festo_command.leg[2] = l3_;

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_festo_command, lib::smb1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb1::ROBOT_NAME.c_str());

}

void swarmitfix_demo_base::move_smb_external(int legs_rotation_, double pkm_rotation_)
{
	sr_ecp_msg->message("swarmitfix_demo_base::move_smb_external");

	lib::smb::motor_command mp_ecp_smb_epos_simple_command;

	mp_ecp_smb_epos_simple_command.base_vs_bench_rotation = legs_rotation_;
	mp_ecp_smb_epos_simple_command.pkm_vs_base_rotation = pkm_rotation_;

	set_next_ecp_state(ecp_mp::smb::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_smb_epos_simple_command, lib::smb1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb1::ROBOT_NAME.c_str());

}

void swarmitfix_demo_base::move_shead_joints(double joint_)
{
	lib::epos::epos_simple_command mp_ecp_shead_epos_simple_command;
	mp_ecp_shead_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	mp_ecp_shead_epos_simple_command.desired_position[0] = joint_;

	set_next_ecp_state(ecp_mp::shead::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_shead_epos_simple_command, lib::shead1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::shead1::ROBOT_NAME.c_str());

}

void swarmitfix_demo_base::move_spkm_joints(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double legA_, double legB_, double legC_, double wrist1_, double wrist2_, double wrist3_)
{
	lib::epos::epos_simple_command mp_ecp_spkm_epos_simple_command;
	mp_ecp_spkm_epos_simple_command.motion_variant = motion_variant_;

	mp_ecp_spkm_epos_simple_command.desired_position[0] = legA_;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = legB_;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = legC_;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = wrist1_;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = wrist2_;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = wrist3_;

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_spkm_epos_simple_command, lib::spkm1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm1::ROBOT_NAME.c_str());

}

void swarmitfix_demo_base::move_spkm_external(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double x_, double y_, double z_, double alpha_, double beta_, double gamma_)
{
	lib::spkm::spkm_epos_simple_command mp_ecp_spkm_epos_simple_command;
	mp_ecp_spkm_epos_simple_command.motion_variant = motion_variant_;
	mp_ecp_spkm_epos_simple_command.pose_specification = lib::spkm::WRIST_XYZ_EULER_ZYZ;
	mp_ecp_spkm_epos_simple_command.estimated_time = 1.2;

	mp_ecp_spkm_epos_simple_command.desired_position[0] = x_;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = y_;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = z_;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = alpha_;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = beta_;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = gamma_;

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_spkm_epos_simple_command, lib::spkm1::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::spkm1::ROBOT_NAME.c_str());

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
