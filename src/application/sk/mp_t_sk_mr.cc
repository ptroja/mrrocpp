// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "ecp_mp_t_sk_mr.h"
#include "mp_t_sk_mr.h"
#include "lib/mrmath/mrmath.h"
#include "ecp_mp/task/ecp_mp_t_tfg.h"
#include "lib/data_port_headers/tfg.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config) {
	return new sk_mr(_config);
}

sk_mr::sk_mr(lib::configurator &_config) :
	task(_config) {
}

void sk_mr::main_task_algorithm(void) {

	sr_ecp_msg->message("New sk_mr series");

	// wybor manipulatora do sterowania na podstawie konfiguracji

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	// ROBOT IRP6_ON_TRACK
	if (config.value<int> ("is_irp6ot_m_active", UI_SECTION)) {
		manipulator_name = lib::ROBOT_IRP6OT_M;
		if (config.value<int> ("is_irp6ot_tfg_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_IRP6OT_TFG;
		} else {
			// TODO: throw
		}
	} else if (config.value<int> ("is_irp6p_m_active", UI_SECTION)) {
		manipulator_name = lib::ROBOT_IRP6P_M;
		if (config.value<int> ("is_irp6p_tfg_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_IRP6P_TFG;
		} else {
			// TODO: throw
		}
	} else {
		// TODO: throw
	}

	// sekwencja generator na wybranym chwytaku

	char tmp_string[MP_2_ECP_STRING_SIZE];

	lib::tfg_command mp_ecp_tfg_command;

	mp_ecp_tfg_command.desired_position = 0.078;

	memcpy(tmp_string, &mp_ecp_tfg_command, sizeof(mp_ecp_tfg_command));

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TFG, (int) 5, tmp_string,
			sizeof(mp_ecp_tfg_command), 1, gripper_name);
	/*
	 run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
	 1, 1, gripper_name, gripper_name);
	 */
	// sekwencja generator na wybranym manipulatorze

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE, (int) 5,
			"", 0, 1, manipulator_name);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, manipulator_name, manipulator_name);

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TFF_NOSE_RUN, (int) 5, "",
			0, 1, manipulator_name);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			2, 2, manipulator_name, gripper_name, manipulator_name,
			gripper_name);

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_EDGE_FOLLOW_FORCE, (int) 5,
			"", 0, 1, manipulator_name);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, manipulator_name, manipulator_name);

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
