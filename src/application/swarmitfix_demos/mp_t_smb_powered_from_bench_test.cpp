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

	// Temporary variables.
	mrrocpp::lib::sbench::power_supply_state ps;
	mrrocpp::lib::sbench::cleaning_state cs;

	// 1st pose.
	power_clean_pose pose1;
	pose1.rotation_pin = pin(4, 4);
	pose1.desired_pin1 = pin(3, 3);
	pose1.desired_pin2 = pin(4, 3);
	pose1.rotation = leg_rotation(1, -1);

	// 2nd pose.
	power_clean_pose pose2;
	pose2.rotation_pin = pin(4, 4);
	pose2.desired_pin1 = pin(4, 3);
	pose2.desired_pin2 = pin(5, 3);
	pose2.rotation = leg_rotation(1, 1);

	// 3rd pose.
	power_clean_pose pose3;
	pose3.rotation_pin = pin(5, 3);
	pose3.desired_pin1 = pin(6, 3);
	pose3.desired_pin2 = pin(6, 4);
	pose3.rotation = leg_rotation(3, 3);

	// 4th pose.
	power_clean_pose pose4;
	pose4.rotation_pin = pin(5, 3);
	pose4.desired_pin1 = pin(4, 3);
	pose4.desired_pin2 = pin(4, 4);
	pose4.rotation = leg_rotation(3, -3);

	// Pull out all legs.
	move_smb_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
	// Wait for given time.
	wait_ms(delay);

	// Work depending on the mode.
	switch (mode){
		default:
			// Set "power walk" mode as default.
		case 0:
			sr_ecp_msg->message("POWER WALK!");
			// Turn power on pins from the 1st pose.
			sr_ecp_msg->message(pose1.get_name());
			ps.set_on(pose1.rotation_pin.row, pose1.rotation_pin.row);
			ps.set_on(pose1.desired_pin1.row, pose1.desired_pin1.row);
			ps.set_on(pose1.desired_pin2.row, pose1.desired_pin2.row);
			while (true) {
				// Power walk!
				bench_move_to_power_pose_with_smb(pose2, delay);
				bench_move_to_power_pose_with_smb(pose3, delay);
				bench_move_to_power_pose_with_smb(pose4, delay);
				bench_move_to_power_pose_with_smb(pose1, delay);
			}
			break;
		case 1:
			sr_ecp_msg->message("POWER WALK! WITH CLEANING");
			// Turn power on pins from the 1st pose.
			sr_ecp_msg->message(pose1.get_name());
			ps.set_on(pose1.rotation_pin.row, pose1.rotation_pin.row);
			ps.set_on(pose1.desired_pin1.row, pose1.desired_pin1.row);
			ps.set_on(pose1.desired_pin2.row, pose1.desired_pin2.row);
			while (true) {
				// Power walk with cleaning.
				bench_move_to_power_pose_with_cleaning_and_smb(pose2, delay, cleaning_time);
				bench_move_to_power_pose_with_cleaning_and_smb(pose3, delay, cleaning_time);
				bench_move_to_power_pose_with_cleaning_and_smb(pose4, delay, cleaning_time);
				bench_move_to_power_pose_with_cleaning_and_smb(pose1, delay, cleaning_time);
			}
			break;
	}

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
