/*!
 * @file mp_t_sbench_test.cpp
 * @brief Class for SBENCH tests methods declaration.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#include "mp_t_sbench_test.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix::sbench_test(_config);
}

namespace swarmitfix {

sbench_test::sbench_test(lib::configurator &config_) :
		demo_base(config_)
{
}

void sbench_test::create_robots()
{
	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}


void sbench_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("sbench_test::main_task_algorithm");

	int mode = config.value <int>("mode");
	unsigned int delay = config.value <int>("delay");
	unsigned int cleaning_time = config.value <int>("cleaning_time");

	// Temporary variables.
	mrrocpp::lib::sbench::power_supply_state ps;
	mrrocpp::lib::sbench::cleaning_state cs;

	// 1st (and also 5th) pose.
	bench_pose pose1;
	pose1.rotation_pin = pin(4, 4);
	pose1.desired_pin1 = pin(3, 3);
	pose1.desired_pin2 = pin(4, 3);
	// 2nd pose.
	bench_pose pose2;
	pose2.rotation_pin = pin(4, 4);
	pose2.desired_pin1 = pin(4, 3);
	pose2.desired_pin2 = pin(5, 3);
	// 3rd pose.
	bench_pose pose3;
	pose3.rotation_pin = pin(5, 3);
	pose3.desired_pin1 = pin(6, 3);
	pose3.desired_pin2 = pin(6, 4);
	// 4th pose.
	bench_pose pose4;
	pose4.rotation_pin = pin(5, 3);
	pose4.desired_pin1 = pin(4, 3);
	pose4.desired_pin2 = pin(4, 4);

	// Work depending on the mode.
	switch (mode)
	{
		default:
			// Set "reset" mode as default.
		case 0:
			sr_ecp_msg->message("RESET BOTH POWER SUPPLY AND CLEANING");
			control_bench_power_supply(ps, delay);
			control_bench_cleaning(cs, delay);
			break;
		case 1:
			sr_ecp_msg->message("POWER SUPPLY");
			while (true) {
				// Power supply.
				ps.set_off(1, 3);
				ps.set_on(1, 1);
				control_bench_power_supply(ps, delay);

				ps.set_off(1, 1);
				ps.set_on(1, 2);
				control_bench_power_supply(ps, delay);

				ps.set_off(1, 2);
				ps.set_on(1, 3);
				control_bench_power_supply(ps, delay);
			}
			break;
		case 2:
			sr_ecp_msg->message("CLEANING");
			while (true) {
				// Cleaning.
				cs.set_on(1, 1);
				control_bench_cleaning(cs, cleaning_time);

				cs.set_off(1, 1);
				cs.set_on(1, 2);
				control_bench_cleaning(cs, cleaning_time);

				cs.set_off(1, 2);
				cs.set_on(1, 3);
				control_bench_cleaning(cs, cleaning_time);

				// Additional wait without blowing.
				cs.set_off(1, 3);
				control_bench_cleaning(cs, cleaning_time);
			}
			break;
		case 3:
			sr_ecp_msg->message("POWER TRAJECTORY");
			// Turn power on the 1st power pose.
			sr_ecp_msg->message(pose1.get_name());
			ps.set_on(pose1.rotation_pin.row, pose1.rotation_pin.row);
			ps.set_on(pose1.desired_pin1.row, pose1.desired_pin1.row);
			ps.set_on(pose1.desired_pin2.row, pose1.desired_pin2.row);
			while (true) {
				bench_move_to_power_pose(pose2, delay);
				bench_move_to_power_pose(pose3, delay);
				bench_move_to_power_pose(pose4, delay);
				bench_move_to_power_pose(pose1, delay);
			}
			break;
		case 4:
			sr_ecp_msg->message("POWER TRAJECTORY WITH CLEANING");
			// Turn power on the 1st power pose.
			sr_ecp_msg->message(pose1.get_name());
			ps.set_on(pose1.rotation_pin.row, pose1.rotation_pin.row);
			ps.set_on(pose1.desired_pin1.row, pose1.desired_pin1.row);
			ps.set_on(pose1.desired_pin2.row, pose1.desired_pin2.row);
			control_bench_power_supply(ps, delay);
			// Move.
			while (true) {
				bench_move_to_power_pose_with_cleaning(pose2, delay, cleaning_time);
				bench_move_to_power_pose_with_cleaning(pose3, delay, cleaning_time);
				bench_move_to_power_pose_with_cleaning(pose4, delay, cleaning_time);
				bench_move_to_power_pose_with_cleaning(pose1, delay, cleaning_time);
			}
			break;
	} //: switch

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
