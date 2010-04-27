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
#include "ecp_mp_t_swarmitfix.h"
#include "mp_t_swarmitfix.h"
#include "lib/single_thread_port.h"
#include "lib/mrmath/mrmath.h"
#include "lib/data_port_headers/epos.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config) {
	return new swarmitfix(_config);
}

swarmitfix::swarmitfix(lib::configurator &_config) :
	task(_config) {
}

void swarmitfix::main_task_algorithm(void) {

	sr_ecp_msg->message("New swarmitfix series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "",
			0, 1, lib::ROBOT_SPKM);
	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "",
			0, 1, lib::ROBOT_SMB);
	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "",
			0, 1, lib::ROBOT_SHEAD);

	double a = 2.88;

	lib::Homog_matrix begining_frame, goal_frame;

	begining_frame.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0.922399,
			0.000199, 0.267710, -2.208626, -0.000010, -2.232600));

	goal_frame.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0.982399,
			0.000199, 0.267710, -1.708626, -0.000010, -2.232600));

	lib::Homog_matrix total_increment_frame = (!begining_frame) * goal_frame;

	lib::Xyz_Angle_Axis_vector total_increment_vector;
	total_increment_frame.get_xyz_angle_axis(total_increment_vector);

	std::cout << total_increment_vector << std::endl;

	lib::Xyz_Angle_Axis_vector third_of_total_increment_vector(
			total_increment_vector[0] / 3, total_increment_vector[1] / 3,
			total_increment_vector[2] / 3, total_increment_vector[3] / 3,
			total_increment_vector[4] / 3, total_increment_vector[5] / 3);

	std::cout << "third_of_total_increment_vector"
			<< third_of_total_increment_vector << std::endl;

	lib::Homog_matrix third_way_frame = begining_frame * lib::Homog_matrix(
			third_of_total_increment_vector);

	/*
	 lib::Xyz_Angle_Axis_vector half_way_aa_vector;
	 half_way_frame.get_xyz_angle_axis(half_way_aa_vector);
	 std::cout << "half_way_aa_vector" << half_way_aa_vector << std::endl;
	 */

	lib::Homog_matrix begining_frame_with_current_translation = begining_frame;
	begining_frame_with_current_translation.set_translation_vector(
			third_way_frame);

	lib::Xyz_Angle_Axis_vector second_step_third_of_total_increment_vector =
			lib::V_tr(!(lib::V_tr(!begining_frame_with_current_translation
					* third_way_frame))) * third_of_total_increment_vector;

	std::cout << "third_of_total_increment_vector"
			<< third_of_total_increment_vector << std::endl;

	third_way_frame = third_way_frame * lib::Homog_matrix(
			second_step_third_of_total_increment_vector);

	std::cout << std::endl << goal_frame << std::endl << third_way_frame
			<< std::endl;

	begining_frame_with_current_translation = begining_frame;
	begining_frame_with_current_translation.set_translation_vector(
			third_way_frame);

	second_step_third_of_total_increment_vector = lib::V_tr(!(lib::V_tr(
			!begining_frame_with_current_translation * third_way_frame)))
			* third_of_total_increment_vector;

	std::cout << "third_of_total_increment_vector"
			<< third_of_total_increment_vector << std::endl;

	lib::Homog_matrix tmp_goal_frame = third_way_frame * lib::Homog_matrix(
			second_step_third_of_total_increment_vector);

	std::cout << std::endl << goal_frame << std::endl << tmp_goal_frame
			<< std::endl;

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

	send_end_motion_to_ecps(1, lib::ROBOT_SPKM);
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

	lib::mp_ecp_epos_gen_parameters epos_params;

	epos_params.dm[4] = 3.7;

	memcpy(tmp_string, &epos_params, sizeof(epos_params));

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_EPOS, (int) 5, tmp_string,
			sizeof(epos_params), 1, lib::ROBOT_SPKM);
	sr_ecp_msg->message("5");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, lib::ROBOT_SPKM, lib::ROBOT_SPKM);

	sr_ecp_msg->message("END");

	send_end_motion_to_ecps(2, lib::ROBOT_SMB, lib::ROBOT_SHEAD);

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
