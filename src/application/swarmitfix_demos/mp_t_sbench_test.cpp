/*!
 * @file mp_t_sbench_test.cpp
 * @brief Class for SBENCH tests methods declaration.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#include "mp_t_sbench_test.h"
#include "ecp_mp_g_sbench.h"

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
/*	int mode = config.value <int> ("mode");
	int delay = config.value <int> ("delay");
*/
/*
	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			robot->ui_ecp_robot->the_robot->power_supply_data_port.data.set_value(i, j, docks[i][j]->isChecked());
		}
	}

	robot->ui_ecp_robot->the_robot->power_supply_data_port.set();
	robot->ui_ecp_robot->the_robot->data_request_port.set_request();
	robot->ui_ecp_robot->execute_motion();
	robot->ui_ecp_robot->the_robot->data_request_port.get();
*/
	sr_ecp_msg->message("demo_base::move_smb_legs");
	mrrocpp::lib::sbench::power_supply_state ps;

	ps.set_value(0, 1, 1);
	ps.set_value(0, 2, 1);
	ps.set_value(0, 3, 1);

	set_next_ecp_state(mrrocpp::ecp_mp::sbench::generator::POWER_SUPPLY_COMMAND, 0, ps, lib::sbench::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::sbench::ROBOT_NAME.c_str());
}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
