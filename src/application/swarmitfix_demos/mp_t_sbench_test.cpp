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
	int mode = config.value <int> ("mode");
	int delay = config.value <int> ("delay");

/*

	ps.set_value(1, 1, 1);
	ps.set_value(1, 2, 1);
	ps.set_value(1, 3, 1);

	set_next_ecp_state(mrrocpp::ecp_mp::sbench::generator::POWER_SUPPLY_COMMAND, 0, ps, lib::sbench::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::sbench::ROBOT_NAME.c_str());
*/
	sr_ecp_msg->message("sbench_test::main_task_algorithm");

	// Temporary variables.
	mrrocpp::lib::sbench::power_supply_state ps;
	mrrocpp::lib::sbench::cleaning_state cs;


	// Work depending on the mode.
	switch (mode){
		default:
			// Set mode "all out and in" as default.
		case 0:
			while (true) {
				sr_ecp_msg->message("POWER SUPPLY");
				// Power supply.
				ps.set_value(1, 3, 0);
				ps.set_value(1, 1, 1);
				control_bench_power_supply(ps, delay);

				ps.set_value(1, 1, 0);
				ps.set_value(1, 2, 1);
				control_bench_power_supply(ps, delay);

				ps.set_value(1, 2, 0);
				ps.set_value(1, 3, 1);
				control_bench_power_supply(ps, delay);
			}
			break;
		case 1:
			while (true) {
				sr_ecp_msg->message("CLEANING");
				// Cleaning.
				cs.set_value(1, 3, 0);
				cs.set_value(1, 1, 1);
				control_bench_cleaning(cs, delay);

				cs.set_value(1, 1, 0);
				cs.set_value(1, 2, 1);
				control_bench_cleaning(cs, delay);

				cs.set_value(1, 2, 0);
				cs.set_value(1, 3, 1);
				control_bench_cleaning(cs, delay);
			}
			break;
	}//: switch

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
