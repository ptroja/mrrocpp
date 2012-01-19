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

	int mode = config.value <int> ("mode");
	int delay = config.value <int> ("delay");

	// Temporary variables.
	mrrocpp::lib::sbench::power_supply_state ps;
	mrrocpp::lib::sbench::cleaning_state cs;


	// Work depending on the mode.
	switch (mode){
		default:
			// Set "reset" mode as default.
		case 0:
			sr_ecp_msg->message("RESET BOTH POWER SUPPLY AND CLEANING");
			control_bench_power_supply(ps, delay);
			control_bench_cleaning(cs, delay);
			break;
		case 1:
			while (true) {
				sr_ecp_msg->message("POWER SUPPLY");
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
			while (true) {
				sr_ecp_msg->message("CLEANING");
				// Cleaning.
				cs.set_on(1, 1);
				control_bench_cleaning(cs, delay);

				cs.set_off(1, 1);
				cs.set_on(1, 2);
				control_bench_cleaning(cs, delay);

				cs.set_off(1, 2);
				cs.set_on(1, 3);
				control_bench_cleaning(cs, delay);

				// Additional wait without blowing.
				cs.set_off(1, 3);
				control_bench_cleaning(cs, delay);
			}
			break;
		case 3:
			while (true) {
				sr_ecp_msg->message("POWER TRAJECTORY");
				// 1st power pose.
				ps.set_on(3, 4);
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				control_bench_power_supply(ps, delay);

				// 2nd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				control_bench_power_supply(ps, delay);

				// 3rd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// 4th power pose.
				ps.set_all_off();
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// 5nd power pose.
				ps.set_all_off();
				ps.set_on(5, 3);
				ps.set_on(6, 3);
				ps.set_on(6, 4);
				control_bench_power_supply(ps, delay);

				// Return.
				// 4th power pose.
				ps.set_all_off();
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// 3rd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// 2nd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				control_bench_power_supply(ps, delay);

				// 1st power pose.
				ps.set_on(3, 4);
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				control_bench_power_supply(ps, delay);
			}
			break;
		case 4:
			while (true) {
				sr_ecp_msg->message("POWER TRAJECTORY WITH CLEANING");
				// 1st power pose.
				ps.set_on(3, 4);
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				control_bench_power_supply(ps, delay);

				// 2nd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				control_bench_power_supply(ps, delay);

				// Ist cleaning pose.
				cs.set_all_off();
				cs.set_on(4, 3);
				cs.set_on(5, 3);
				control_bench_cleaning(cs, delay);

				// 3rd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// 4th power pose.
				ps.set_all_off();
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// IInd cleaning pose.
				cs.set_all_off();
				cs.set_on(6, 3);
				cs.set_on(6, 4);
				control_bench_cleaning(cs, delay);

				// 5nd power pose.
				ps.set_all_off();
				ps.set_on(5, 3);
				ps.set_on(6, 3);
				ps.set_on(6, 4);
				control_bench_power_supply(ps, delay);

				// Return.
				// 4th power pose.
				ps.set_all_off();
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// IIIrd cleaning pose.
				cs.set_all_off();
				cs.set_on(4, 3);
				cs.set_on(5, 3);
				control_bench_cleaning(cs, delay);

				// 3rd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				ps.set_on(5, 3);
				control_bench_power_supply(ps, delay);

				// 2nd power pose.
				ps.set_all_off();
				ps.set_on(4, 4);
				control_bench_power_supply(ps, delay);

				// IVth cleaning pose.
				cs.set_all_off();
				cs.set_on(3, 4);
				cs.set_on(4, 3);
				control_bench_cleaning(cs, delay);

				// 1st power pose.
				ps.set_on(3, 4);
				ps.set_on(4, 4);
				ps.set_on(4, 3);
				control_bench_power_supply(ps, delay);
			}
			break;
	}//: switch

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
