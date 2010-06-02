
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "lib/mrmath/mrmath.h"
#include "ecp_mp/task/ecp_mp_t_tfg.h"
#include "lib/data_port_headers/tfg.h"
#include "mp_t_tfg_graspit.h"
#include "ecp_mp_t_graspit.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

graspit::graspit(lib::configurator &_config) :
	task(_config) {

	trgraspit = new ecp_mp::transmitter::TRGraspit(ecp_mp::transmitter::TRANSMITTER_GRASPIT,"[transmitter_graspit]",*this);
}

void graspit::main_task_algorithm(void) {

	sr_ecp_msg->message("START GRASP");

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	int port=config.value<int>("graspit_port","[transmitter_graspit]");
	std::string node_name=config.value<std::string>("graspit_node_name","[transmitter_graspit]");

	//get the data from GraspIt
	trgraspit->TRconnect(node_name.c_str(), port);
	trgraspit->t_read();
	trgraspit->TRdisconnect();

	//mrroc++ kinematics,
	//1 affects 2
	//2 affects 3
	trgraspit->from_va.grasp_joint[2] += trgraspit->from_va.grasp_joint[1];
	trgraspit->from_va.grasp_joint[3] += trgraspit->from_va.grasp_joint[2];

	trgraspit->from_va.grasp_joint[9] += trgraspit->from_va.grasp_joint[8];
	trgraspit->from_va.grasp_joint[10] += trgraspit->from_va.grasp_joint[9];

	//synchro with GraspIt
	//trgraspit->from_va.grasp_joint[0] ;
	trgraspit->from_va.grasp_joint[1] -= 1.542;
	//trgraspit->from_va.grasp_joint[2] ;
	//trgraspit->from_va.grasp_joint[3] ;
	trgraspit->from_va.grasp_joint[4] += 4.712;
	//trgraspit->from_va.grasp_joint[5] ;
	trgraspit->from_va.grasp_joint[6] = (45.5 - trgraspit->from_va.grasp_joint[6]) * 2;
	trgraspit->from_va.grasp_joint[6] /= 1000;

	//trgraspit->from_va.grasp_joint[7] ;
	trgraspit->from_va.grasp_joint[8] -= 1.542;
	//trgraspit->from_va.grasp_joint[9] ;
	//trgraspit->from_va.grasp_joint[10] ;
	trgraspit->from_va.grasp_joint[11] += 4.712;
	//trgraspit->from_va.grasp_joint[12] ;
	trgraspit->from_va.grasp_joint[13] = (45.5 - trgraspit->from_va.grasp_joint[13]) * 2;
	trgraspit->from_va.grasp_joint[13] /= 1000;



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

	char tmp_string[MP_2_ECP_STRING_SIZE];

	lib::tfg_command mp_ecp_tfg_command;

	mp_ecp_tfg_command.desired_position = trgraspit->from_va.grasp_joint[6];

	memcpy(tmp_string, &mp_ecp_tfg_command, sizeof(mp_ecp_tfg_command));



	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TFG, (int) 5, tmp_string,
			sizeof(mp_ecp_tfg_command), 1, gripper_name);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, gripper_name, gripper_name);

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_IRP6, (int) 5, "",
			0, 1, manipulator_name);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, manipulator_name, manipulator_name);



	sr_ecp_msg->message("END GRASP");
}

task* return_created_mp_task(lib::configurator &_config) {
	return new graspit(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
