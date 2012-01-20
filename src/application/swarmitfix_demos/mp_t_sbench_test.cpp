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
	int mode = config.value <int> ("mode");
	switch (mode){
		default:
			// Set "reset" mode as default.
		case 0:
			sr_ecp_msg->message("RESET BOTH POWER SUPPLY AND CLEANING");
			break;
		case 1:
			sr_ecp_msg->message("PIN POWER MOVE");
			break;
		case 2:
			sr_ecp_msg->message("CLEANING MOVE");
			break;
		case 3:
			sr_ecp_msg->message("POWER MOVE!");
			break;
		case 4:
			sr_ecp_msg->message("POWER MOVE! WITH CLEANING");
			break;
	}//: switch

}

void sbench_test::create_robots()
{
	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}


void sbench_test::main_task_algorithm(void)
{
	using namespace mrrocpp::lib::sbench;

	sr_ecp_msg->message("sbench_test::main_task_algorithm");

	int mode = config.value <int>("mode");
	unsigned int delay = config.value <int>("delay");
	unsigned int cleaning_time = config.value <int>("cleaning_time");

	// Temporary variables.
	mrrocpp::lib::sbench::power_supply_state ps;
	mrrocpp::lib::sbench::cleaning_state cs;

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
			sr_ecp_msg->message("PIN POWER MOVE");
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
			sr_ecp_msg->message("CLEANING MOVE");
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
			sr_ecp_msg->message("POWER MOVE!");
			// Turn power on 1st pose pins.
			sr_ecp_msg->message(pose1.get_description());
			ps.set_on(pose1);
			control_bench_power_supply(ps, delay);
			while (true) {
				bench_execute_power_move(move1, delay);
				bench_execute_power_move(move2, delay);
				bench_execute_power_move(move3, delay);
				bench_execute_power_move(move4, delay);
				sr_ecp_msg->message("Finished - waiting for 3s");
				wait_ms(3000);
			}
			break;
		case 4:
			sr_ecp_msg->message("POWER MOVE! WITH CLEANING");
			// Turn power on the 1st power pose.
			sr_ecp_msg->message(pose1.get_description());
			sr_ecp_msg->message(pose1.get_description());
			ps.set_on(pose1);
			control_bench_power_supply(ps, delay);
			// Move.
			while (true) {
				bench_execute_power_move_with_cleaning(move1, delay, cleaning_time);
				bench_execute_power_move_with_cleaning(move2, delay, cleaning_time);
				bench_execute_power_move_with_cleaning(move3, delay, cleaning_time);
				bench_execute_power_move_with_cleaning(move4, delay, cleaning_time);
				sr_ecp_msg->message("Finished - waiting for 3s");
				wait_ms(3000);
			}
			break;
	} //: switch

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
