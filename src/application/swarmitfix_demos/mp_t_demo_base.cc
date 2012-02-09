#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"

#include "mp_t_demo_base.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "generator/ecp/transparent/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"
#include "ecp_mp_g_smb.h"
#include "ecp_mp_g_shead.h"
#include "ecp_mp_g_sbench.h"

namespace mrrocpp {
namespace mp {
namespace task {
namespace swarmitfix {


demo_base::demo_base(lib::configurator &config_) :
		task(config_)
{

}

void demo_base::smb_stan_on_one_leg(int leg_number_) {
	sr_ecp_msg->message("demo_base::smb_stan_on_the_leg");
	// Pull all legs in except the one around which the rotation_ will be performed.
	switch (leg_number_)
	{
		case 1:
			sr_ecp_msg->message("demo_base::rotate_smb: OUT, IN, IN");
			smb_pull_legs(lib::smb::OUT, lib::smb::IN, lib::smb::IN);
			break;
		case 2:
			sr_ecp_msg->message("demo_base::rotate_smb: IN, OUT, IN");
			smb_pull_legs(lib::smb::IN, lib::smb::OUT, lib::smb::IN);
			break;
		case 3:
			sr_ecp_msg->message("demo_base::rotate_smb: IN, IN, OUT");
			smb_pull_legs(lib::smb::IN, lib::smb::IN, lib::smb::OUT);
			break;
		default:
			break;
	}
}

void demo_base::rotate_smb(int leg_number_, int rotation_)
{
	sr_ecp_msg->message("demo_base::rotate_smb");

	// Stand on one leg.
	smb_stan_on_one_leg(leg_number_);

	// Rotate around the leg - the SPKM rotation is set to zero.
	smb_rotate_external(rotation_, 0);

	// Pull all legs out.
	sr_ecp_msg->message("demo_base::rotate_smb: OUT, OUT, OUT");
	smb_pull_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
}


void demo_base::move_to_pose_and_return(const lib::Xyz_Euler_Zyz_vector & support_pose_, const lib::Xyz_Euler_Zyz_vector & inter_pose_, double smb_joint_, double shead_joint_)
{
	// Move SMB and SPKM to pose.
	smb_rotate_external(0, smb_joint_);
	// Support interpose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, inter_pose_);
	// Rotate shead.
	move_shead_joints(shead_joint_);
	// Support.
	move_spkm_external(lib::epos::OPERATIONAL, support_pose_);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	// Support interpose.
	move_spkm_external(lib::epos::OPERATIONAL, inter_pose_);
	// Neutral.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, lib::Xyz_Euler_Zyz_vector(0.15, -0.035, 0.405, 0, -0.92, 0));
}

void demo_base::smb_pull_legs(lib::smb::FESTO_LEG l1_, lib::smb::FESTO_LEG l2_, lib::smb::FESTO_LEG l3_)
{
	sr_ecp_msg->message("demo_base::move_smb_legs");
	lib::smb::festo_command_td mp_ecp_festo_command;

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.undetachable[i] = false;
	}

	mp_ecp_festo_command.leg[0] = l1_;
	mp_ecp_festo_command.leg[1] = l2_;
	mp_ecp_festo_command.leg[2] = l3_;

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_festo_command, smb_robot_name);
	wait_for_task_termination(false, smb_robot_name);

}

void demo_base::smb_rotate_external(int legs_rotation_, double pkm_rotation_)
{
	sr_ecp_msg->message("demo_base::move_smb_external");

	lib::smb::motor_command mp_ecp_smb_epos_simple_command;

	mp_ecp_smb_epos_simple_command.base_vs_bench_rotation = legs_rotation_;
	mp_ecp_smb_epos_simple_command.pkm_vs_base_rotation = pkm_rotation_;

	set_next_ecp_state(ecp_mp::smb::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_smb_epos_simple_command, smb_robot_name);
	wait_for_task_termination(false, smb_robot_name);

}

void demo_base::move_shead_joints(double joint_)
{
	lib::epos::epos_simple_command mp_ecp_shead_epos_simple_command;
	mp_ecp_shead_epos_simple_command.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	mp_ecp_shead_epos_simple_command.desired_position[0] = joint_;

	set_next_ecp_state(ecp_mp::shead::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_shead_epos_simple_command, shead_robot_name);
	wait_for_task_termination(false, shead_robot_name);

}

void demo_base::move_spkm_joints(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double legA_, double legB_, double legC_, double wrist1_, double wrist2_, double wrist3_)
{
	lib::epos::epos_simple_command mp_ecp_spkm_epos_simple_command;
	mp_ecp_spkm_epos_simple_command.motion_variant = motion_variant_;

	mp_ecp_spkm_epos_simple_command.desired_position[0] = legA_;
	mp_ecp_spkm_epos_simple_command.desired_position[1] = legB_;
	mp_ecp_spkm_epos_simple_command.desired_position[2] = legC_;
	mp_ecp_spkm_epos_simple_command.desired_position[3] = wrist1_;
	mp_ecp_spkm_epos_simple_command.desired_position[4] = wrist2_;
	mp_ecp_spkm_epos_simple_command.desired_position[5] = wrist3_;

	set_next_ecp_state(ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND, 0, mp_ecp_spkm_epos_simple_command, spkm_robot_name);
	wait_for_task_termination(false, spkm_robot_name);

}

void demo_base::move_spkm_external(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, const lib::Xyz_Euler_Zyz_vector & pose_)
{
	lib::spkm::spkm_epos_simple_command mp_ecp_spkm_epos_simple_command;
	mp_ecp_spkm_epos_simple_command.motion_variant = motion_variant_;
	mp_ecp_spkm_epos_simple_command.pose_specification = lib::spkm::POSE_SPECIFICATION::WRIST_XYZ_EULER_ZYZ;
	mp_ecp_spkm_epos_simple_command.estimated_time = 1.2;

	mp_ecp_spkm_epos_simple_command.desired_position[0] = pose_(0);
	mp_ecp_spkm_epos_simple_command.desired_position[1] = pose_(1);
	mp_ecp_spkm_epos_simple_command.desired_position[2] = pose_(2);
	mp_ecp_spkm_epos_simple_command.desired_position[3] = pose_(3);
	mp_ecp_spkm_epos_simple_command.desired_position[4] = pose_(4);
	mp_ecp_spkm_epos_simple_command.desired_position[5] = pose_(5);

	std::cout<<" spkm_robot_name:" << spkm_robot_name <<" -> set_next_ecp_state\n";
	set_next_ecp_state(ecp_mp::spkm::generator::ECP_EXTERNAL_EPOS_COMMAND, 0, mp_ecp_spkm_epos_simple_command, spkm_robot_name);
	std::cout<<" spkm_robot_name:" << spkm_robot_name <<" -> wait_for_task_termination\n";
	wait_for_task_termination(false, spkm_robot_name);
	std::cout<<" spkm_robot_name:" << spkm_robot_name <<" -> !done!\n";
}

void demo_base::control_bench_power_supply(const mrrocpp::lib::sbench::power_supply_state & ps_, int delay_) {
	this->sr_ecp_msg->message("demo_base::control_bench_power_supply");
	this->sr_ecp_msg->message(ps_.display());
	set_next_ecp_state(mrrocpp::ecp_mp::sbench::generator::POWER_SUPPLY_COMMAND, 0, ps_, lib::sbench::ROBOT_NAME);
	wait_for_task_termination(false, lib::sbench::ROBOT_NAME);
	wait_ms(delay_);
}

void demo_base::control_bench_cleaning(const mrrocpp::lib::sbench::cleaning_state & cs_, int delay_) {
	this->sr_ecp_msg->message("demo_base::control_bench_cleaning");
	this->sr_ecp_msg->message(cs_.display());
	set_next_ecp_state(mrrocpp::ecp_mp::sbench::generator::CLEANING_COMMAND, 0, cs_, lib::sbench::ROBOT_NAME);
	wait_for_task_termination(false, lib::sbench::ROBOT_NAME);
	wait_ms(delay_);
}

void demo_base::bench_execute_power_move(const power_smb_move & move_, unsigned int delay_) {
	sr_ecp_msg->message(move_.get_description());

	// Bench state.
	// TODO: it should be read from current bench state!!
	mrrocpp::lib::sbench::power_supply_state power;

	// Turn power only on the rotation pin.
	power.set_off(move_.start_pose);
	power.set_on(move_.rotation_pin);
	this->sr_ecp_msg->message(std::string("power on pin:")+move_.rotation_pin.get_description());
	control_bench_power_supply(power, delay_);

	// Power the final pose.
	power.set_on(move_.final_pose);
	control_bench_power_supply(power, delay_);
}


void demo_base::bench_execute_power_move_with_cleaning(const power_smb_move & move_, unsigned int delay_, unsigned int cleaning_time_)
{
	sr_ecp_msg->message(move_.get_description());

	// Bench power and cleaning states.
	// TODO: they should be read from current bench state!!
	mrrocpp::lib::sbench::power_supply_state power;
	mrrocpp::lib::sbench::cleaning_state cleaning;

	// Turn power only on the rotation pin.
	power.set_off(move_.start_pose);
	power.set_on(move_.rotation_pin);
	this->sr_ecp_msg->message(std::string("power on pin:")+move_.rotation_pin.get_description());
	control_bench_power_supply(power, delay_);

	// Turn the cleaning on two pins of the desired pose.
	cleaning.set_on(move_.final_pose);
	cleaning.set_off(move_.rotation_pin);
	control_bench_cleaning(cleaning, cleaning_time_);

	// Turn of the cleaning.
	cleaning.set_off(move_.final_pose);
	control_bench_cleaning(cleaning, 0);

	// Power the final pose.
	power.set_on(move_.final_pose);
	control_bench_power_supply(power, delay_);
}

void demo_base::smb_execute_power_move(const power_smb_move & move_, unsigned int delay_) {
	sr_ecp_msg->message(move_.get_description());

	// Bench state.
	// TODO: it should be read from current bench state!!
	mrrocpp::lib::sbench::power_supply_state power;

	// Turn power only on the rotation pin.
	power.set_off(move_.start_pose);
	power.set_on(move_.rotation_pin);
	this->sr_ecp_msg->message(std::string("power on pin:")+move_.rotation_pin.get_description());
	control_bench_power_supply(power, delay_);

	// Rotate on the leg.
	rotate_smb(move_.rotation_leg.leg, move_.rotation_leg.rotation);
	wait_ms(delay_);

	// Power the final pose.
	power.set_on(move_.final_pose);
	control_bench_power_supply(power, delay_);
}

void demo_base::smb_execute_power_move_with_cleaning(const power_smb_move & move_, unsigned int delay_, unsigned int cleaning_time_) {
	sr_ecp_msg->message(move_.get_description());

	// Bench power and cleaning states.
	// TODO: they should be read from current bench state!!
	mrrocpp::lib::sbench::power_supply_state power;
	mrrocpp::lib::sbench::cleaning_state cleaning;

	// Turn power only on the rotation pin.
	power.set_off(move_.start_pose);
	power.set_on(move_.rotation_pin);
	this->sr_ecp_msg->message(std::string("power on pin:")+move_.rotation_pin.get_description());
	control_bench_power_supply(power, 0);

	// Stand on one leg and rotate around it - the SPKM rotation is set to zero
	smb_stan_on_one_leg(move_.rotation_leg.leg);
	smb_rotate_external(move_.rotation_leg.rotation, 0);
	wait_ms(delay_);

	// Turn the cleaning on two pins of the desired pose.
	cleaning.set_on(move_.final_pose);
	cleaning.set_off(move_.rotation_pin);
	control_bench_cleaning(cleaning, 0);

	// Stand on all legs.
	smb_pull_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
	// Turn off the cleaning.
	cleaning.set_off(move_.final_pose);
	control_bench_cleaning(cleaning, 0);

	// Power the final pose.
	power.set_on(move_.final_pose);
	control_bench_power_supply(power, delay_);
}



} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
