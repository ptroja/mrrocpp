/*!
 * @file mp_t_smb_powered_from_bench_test.cpp
 * @brief Class for powered SMB tests methods declaration.
 *
 * @date Jan 17, 2012
 * @author tkornuta
 */

#include "mp_t_smb_powered_from_bench_test.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix::smb_powered_from_bench_test(_config);
}

namespace swarmitfix {

smb_powered_from_bench_test::smb_powered_from_bench_test(lib::configurator &config_) :
		demo_base(config_)
{
	if (IS_MP_ROBOT_ACTIVE (smb1)){
		smb_robot_name = lib::smb1::ROBOT_NAME;
	} else {//if (IS_MP_ROBOT_ACTIVE (smb2)){
		smb_robot_name = lib::smb2::ROBOT_NAME;
	} // else ?

	int mode = config.value <int> ("mode");
	switch (mode){
		default:
			// Set "power walk" mode as default.
		case 0:
			sr_ecp_msg->message("POWER MOVE!");
			break;
		case 1:
			sr_ecp_msg->message("POWER MOVE! WITH CLEANING");
			break;
	}//: switch
}

void smb_powered_from_bench_test::create_robots()
{
	// Activate SMB robot (depending on the configuration settings).
	if (smb_robot_name == lib::smb1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(smb1)
	} else {
		ACTIVATE_MP_ROBOT(smb2)
	}
	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}


void smb_powered_from_bench_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("smb_powered_from_bench_test::main_task_algorithm");
	int mode = config.value <int> ("mode");
	unsigned int delay = config.value <unsigned int> ("delay");
	unsigned int cleaning_time = config.value <int>("cleaning_time");

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

	// Pull out all legs.
	smb_pull_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
	// Wait for given time.
	wait_ms(delay);

	// Turn power on 1st pose pins.
	sr_ecp_msg->message(pose1.get_description());
	mrrocpp::lib::sbench::power_supply_state ps;
	ps.set_on(pose1);
	control_bench_power_supply(ps, delay);

	// Work depending on the mode.
	switch (mode){
		default:
			// Set "power walk" mode as default.
		case 0:
			sr_ecp_msg->message("POWER MOVE!");
			while (true) {
				// Power walk!
				smb_execute_power_move(move1, delay);
				smb_execute_power_move(move2, delay);
				smb_execute_power_move(move3, delay);
				smb_execute_power_move(move4, delay);
				sr_ecp_msg->message("Finished - waiting for 3s");
				wait_ms(3000);
			}
			break;
		case 1:
			sr_ecp_msg->message("POWER MOVE! WITH CLEANING");
			while (true) {
				// Power walk with cleaning.
				smb_execute_power_move_with_cleaning(move1, delay, cleaning_time);
				smb_execute_power_move_with_cleaning(move2, delay, cleaning_time);
				smb_execute_power_move_with_cleaning(move3, delay, cleaning_time);
				smb_execute_power_move_with_cleaning(move4, delay, cleaning_time);
				sr_ecp_msg->message("Finished - waiting for 3s");
				wait_ms(3000);
			}
			break;
	}

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
