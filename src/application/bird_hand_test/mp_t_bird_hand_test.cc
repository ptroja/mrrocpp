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
#include "ecp_mp_t_bird_hand_test.h"
#include "mp_t_bird_hand_test.h"
#include "lib/single_thread_port.h"
#include "lib/mrmath/mrmath.h"
#include "lib/data_port_headers/bird_hand.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config) {
	return new bird_hand_test(_config);
}

bird_hand_test::bird_hand_test(lib::configurator &_config) :
	task(_config) {
}

void bird_hand_test::main_task_algorithm(void) {

	sr_ecp_msg->message("New bird_hand_test series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "",
			0, 1, lib::ROBOT_BIRD_HAND);

	double a = 2.88;

	std::string astring;

	astring = "11";
	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << a;
	astring = ss.str();

	lib::single_thread_port<int> int_port("int_port_label");
	lib::single_thread_port<int>* int_port_from_manager;

	lib::single_thread_port_manager port_manager;

	port_manager.add_port(&int_port);

	int_port_from_manager = port_manager.get_port<int> ("int_port_label");

	int int_port_data_input = 16;
	int int_port_data_output;

	int_port_from_manager->set(int_port_data_input);
	int_port_data_output = int_port_from_manager->get();

	ss << " " << int_port_data_output;

	sr_ecp_msg->message(ss.str().c_str());

	send_end_motion_to_ecps(1, lib::ROBOT_BIRD_HAND);
	/*
	 sr_ecp_msg->message("2");
	 set_next_ecps_state((int) ecp_mp::task::ECP_GEN_SLEEP, (int) 5, "",  0,1,
	 lib::ROBOT_SPKM);
	 sr_ecp_msg->message("3");
	 run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
	 1, 1, lib::ROBOT_SPKM, lib::ROBOT_SPKM);
	 */
	sr_ecp_msg->message("4");

	char tmp_string[300];

	lib::bird_hand_gen_parameters bird_hand_params;

	bird_hand_params.dm[4] = 3.7;

	memcpy(tmp_string, &bird_hand_params, sizeof(bird_hand_params));

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_BIRD_HAND, (int) 5,
			tmp_string, sizeof(bird_hand_params), 1, lib::ROBOT_BIRD_HAND);
	sr_ecp_msg->message("5");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, lib::ROBOT_BIRD_HAND, lib::ROBOT_BIRD_HAND);

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
