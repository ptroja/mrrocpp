/*!
 * @file mp_t_agent1_demo.cpp
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#include "mp_t_agent1_demo.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix::agent1_demo(_config);
}

namespace swarmitfix {

agent1_demo::agent1_demo(lib::configurator &config_) :
		demo_base(config_)
{
	// SMB.
	if (IS_MP_ROBOT_ACTIVE (smb1)) {
		smb_robot_name = lib::smb1::ROBOT_NAME;
	} else if (IS_MP_ROBOT_ACTIVE (smb2)) {
		smb_robot_name = lib::smb2::ROBOT_NAME;
	} else {
		// TODO: throw - but what??
	}
	// SPKM.
	if (IS_MP_ROBOT_ACTIVE (spkm1)) {
		spkm_robot_name = lib::spkm1::ROBOT_NAME;
	} else if (IS_MP_ROBOT_ACTIVE (spkm2)) {
		spkm_robot_name = lib::spkm2::ROBOT_NAME;
	} else {
		// TODO: throw - but what??
	}
	// SHEAD.
	if (IS_MP_ROBOT_ACTIVE (shead1)) {
		shead_robot_name = lib::shead1::ROBOT_NAME;
	} else if (IS_MP_ROBOT_ACTIVE (smb2)) {
		shead_robot_name = lib::shead2::ROBOT_NAME;
	} else {
		// TODO: throw - but what??
	}

}

void agent1_demo::create_robots()
{
	// Activate robots (depending on the configuration settings).
	// SMB.
	if (smb_robot_name == lib::smb1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(smb1)
	} else {
		ACTIVATE_MP_ROBOT(smb2)
	}
	// SPKM.
	if (spkm_robot_name == lib::spkm1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(spkm1)
	} else {
		ACTIVATE_MP_ROBOT(spkm2)
	}
	// SHEAD.
	if (shead_robot_name == lib::shead1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(shead1)
	} else {
		ACTIVATE_MP_ROBOT(shead2)
	}
	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}

void agent1_demo::main_task_algorithm(void)
{
	sr_ecp_msg->message("smb_powered_from_bench_test::main_task_algorithm");
	int mode = config.value <int>("mode");
	unsigned int delay = config.value <unsigned int>("delay");
	//unsigned int cleaning_time = config.value <int>("cleaning_time");

	// 1st pose.
	bench_pose pose1;
	pose1.pins[0] = pin(4, 4);
	pose1.pins[1] = pin(3, 3);
	pose1.pins[2] = pin(4, 3);

	// 2nd pose.
	bench_pose pose2;
	pose2.pins[0] = pin(4, 4);
	pose2.pins[1] = pin(4, 3);
	pose2.pins[2] = pin(5, 3);

	// 3rd pose.
	bench_pose pose3;
	pose3.pins[0] = pin(6, 3);
	pose3.pins[1] = pin(6, 4);
	pose3.pins[2] = pin(5, 3);

	// Power trajectory.
	power_smb_move move1 = power_smb_move(pose1, pose2, pkm_leg_rotation(1, 1));
	power_smb_move move2 = power_smb_move(pose2, pose3, pkm_leg_rotation(3, 3));
	power_smb_move move3 = power_smb_move(pose3, pose2, pkm_leg_rotation(3, -3));
	power_smb_move move4 = power_smb_move(pose2, pose1, pkm_leg_rotation(1, -1));

	// Support poses.
	/*
	 neutral pose OK
	 tool:  -0.1412 -0.035 0.5768 3.1416 0.137 0
	 wrist: 0.15 -0.035 0.405 0 -0.92 0
	 */
	lib::Xyz_Euler_Zyz_vector neutral_pose = lib::Xyz_Euler_Zyz_vector(0.15, -0.035, 0.405, 0, -0.92, 0);

	/*
	 Podparcie nr 1: smb rot = 1
	 tool:  -0.33 0.0012 0.625 -0.8193 0.029 -2.3437
	 wrist: -0.0693 0 0.4097 0 -0.763 -0.03

	 odejście:
	 tool:  -0.33 0 0.59 3.1416 0.02 0
	 wrist: -0.0609 0 0.3853 0 -0.803 0
	 */
	lib::Xyz_Euler_Zyz_vector support_pose1 = lib::Xyz_Euler_Zyz_vector(-0.0693, 0, 0.4097, 0, -0.763, -0.03);
	lib::Xyz_Euler_Zyz_vector inter_pose1 = lib::Xyz_Euler_Zyz_vector(-0.0609, 0, 0.3853, 0, -0.803, 0);

	/*
	 Podparcie 2: smb rot = 0, legs out
	 tool:   -0.2919 0 0.626 -3.1416 -0.03 0
	 wrist:  -0.0333 0 0.4081 0 -0.753 0

	 odejście z podparcia nr 1:
	 tool:  -0.2919 0 0.6 -3.1416 0 0
	 wrist: -0.0269 0 0.39 0 -0.783 0
	 */
	lib::Xyz_Euler_Zyz_vector support_pose2 = lib::Xyz_Euler_Zyz_vector(-0.0333, 0, 0.4081, 0, -0.753, 0);
	lib::Xyz_Euler_Zyz_vector inter_pose2 = lib::Xyz_Euler_Zyz_vector(-0.0269, 0, 0.39, 0, -0.783, 0);

	/*
	 Podparcie nr 3: smb rot = -1
	 tool: -0.3691 0 0.627 3.1416 -0.05 0
	 wrist: -0.1149 0 0.404 0 -0.733 0

	 odejście:
	 tool:  -0.3691 0 0.5847 3.1416 0.02 0
	 wrist: -0.1 0 0.38 0 -0.803 0
	 */
	lib::Xyz_Euler_Zyz_vector support_pose3 = lib::Xyz_Euler_Zyz_vector(-0.1149, 0, 0.404, 0, -0.733, 0);
	lib::Xyz_Euler_Zyz_vector inter_pose3 = lib::Xyz_Euler_Zyz_vector(-0.1, 0, 0.38, 0, -0.803, 0);

	// Pull out all legs.
//	smb_pull_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
	// Wait for given time.
//	wait_ms(delay);

	// Turn power on 1st pose pins.
	sr_ecp_msg->message(pose1.get_description());
	mrrocpp::lib::sbench::power_supply_state ps;
	ps.set_on(pose1);
	control_bench_power_supply(ps, delay);

	if ((mode == 0) || (mode == 2)) {
		// Move to the *neutral* PKM pose.
		move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, neutral_pose);

		// Move shead to synchro position.
		move_shead_joints(0.0);

		// Move to first pose.
		move_to_pose_and_return(support_pose1, inter_pose1, 1, 0.5);
	}

	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move1, delay);
	}

	if ((mode == 0) || (mode == 2)) {
		move_to_pose_and_return(support_pose2, inter_pose2, 0, -0.5);
		move_to_pose_and_return(support_pose3, inter_pose3, -1, 0);
	}

	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move2, delay);
	}
	if ((mode == 0) || (mode == 2)) {
		move_to_pose_and_return(support_pose2, inter_pose2, 0, -0.5);
		move_to_pose_and_return(support_pose1, inter_pose1, 1, 0.5);
	}
	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move3, delay);
	}
	if ((mode == 0) || (mode == 2)) {
		move_to_pose_and_return(support_pose3, inter_pose3, -1, 0);
	}
	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move4, delay);
	}
	sr_ecp_msg->message("Task finished");

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
