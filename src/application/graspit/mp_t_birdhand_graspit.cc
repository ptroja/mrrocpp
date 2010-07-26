#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "base/mp/mp.h"
#include "lib/mrmath/mrmath.h"
#include "lib/data_port_headers/bird_hand.h"
#include "mp_t_birdhand_graspit.h"
#include "ecp_mp_t_graspit.h"
#include "ecp_mp_g_birdhand.h"
#include "robot/bird_hand/bird_hand_const.h"
#include <iostream>
#include <string>
#include <sstream>
#include "robot/irp6ot_m/irp6ot_m_const.h"
#include "robot/irp6p_m/irp6p_m_const.h"

namespace mrrocpp {
namespace mp {
namespace task {

graspit::graspit(lib::configurator &_config) :
	task(_config)
{
	trgraspit = new ecp_mp::transmitter::TRGraspit(ecp_mp::transmitter::TRANSMITTER_GRASPIT, "[transmitter_graspit]", *this);
}

void graspit::main_task_algorithm(void)
{

	//delta for hand position control
	const double delta_pos = 0.1;

	sr_ecp_msg->message("START GRASP");

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	int port = config.value <int> ("graspit_port", "[transmitter_graspit]");
	std::string node_name = config.value <std::string> ("graspit_node_name", "[transmitter_graspit]");

	//get the data from GraspIt
	trgraspit->TRconnect(node_name.c_str(), port);
	trgraspit->t_read();
	trgraspit->TRdisconnect();

	//mrroc++ kinematics,
	//1 affects 2
	//2 affects 3
	trgraspit->from_va.grasp_joint[2] += trgraspit->from_va.grasp_joint[1];
	trgraspit->from_va.grasp_joint[3] += trgraspit->from_va.grasp_joint[2];

	trgraspit->from_va.grasp_joint[8] += trgraspit->from_va.grasp_joint[7];
	trgraspit->from_va.grasp_joint[9] += trgraspit->from_va.grasp_joint[8];

	//IRp6 synchro with GraspIt
	//trgraspit->from_va.grasp_joint[0] ;
	trgraspit->from_va.grasp_joint[1] -= 1.542;
	//trgraspit->from_va.grasp_joint[2] ;
	//trgraspit->from_va.grasp_joint[3] ;
	trgraspit->from_va.grasp_joint[4] += 4.712;
	//trgraspit->from_va.grasp_joint[5] ;

	//trgraspit->from_va.grasp_joint[6] ;
	trgraspit->from_va.grasp_joint[7] -= 1.542;
	//trgraspit->from_va.grasp_joint[8] ;
	//trgraspit->from_va.grasp_joint[9] ;
	trgraspit->from_va.grasp_joint[10] += 4.712;
	//trgraspit->from_va.grasp_joint[11] ;

	//Bird Hand synchro with GraspIt
	//already ok

	// ROBOT IRP6_ON_TRACK
	if (config.value <int> ("is_irp6ot_m_active", UI_SECTION)) {
		manipulator_name = lib::ROBOT_IRP6OT_M;
		if (config.value <int> ("is_bird_hand_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_BIRD_HAND;
		} else {
			// TODO: throw
		}
	} else if (config.value <int> ("is_irp6p_m_active", UI_SECTION)) {
		manipulator_name = lib::ROBOT_IRP6P_M;
		if (config.value <int> ("is_bird_hand_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_BIRD_HAND;
		} else {
			// TODO: throw
		}
	} else {
		// TODO: throw
	}

	char tmp_string1[MP_2_ECP_STRING_SIZE];
	char tmp_string2[MP_2_ECP_STRING_SIZE];

	struct _irp6{
		double joint[6];
	} mp_ecp_irp6_command;
	lib::bird_hand_command mp_ecp_bird_hand_command;

	//middle IRp6 position from GraspIt
	for (int i=0; i<6; ++i)
		mp_ecp_irp6_command.joint[i] = trgraspit->from_va.grasp_joint[i+6];
	//we want opened hand
	mp_ecp_bird_hand_command.thumb_f[0].desired_position = 0.5;
	mp_ecp_bird_hand_command.thumb_f[1].desired_position = 0.4;
	mp_ecp_bird_hand_command.index_f[0].desired_position = 0.0;
	mp_ecp_bird_hand_command.index_f[1].desired_position = 0.5;
	mp_ecp_bird_hand_command.index_f[2].desired_position = 0.4;
	mp_ecp_bird_hand_command.ring_f[0].desired_position = 0.0;
	mp_ecp_bird_hand_command.ring_f[1].desired_position = 0.5;
	mp_ecp_bird_hand_command.ring_f[2].desired_position = 0.4;

	memcpy(tmp_string1, &mp_ecp_bird_hand_command, sizeof(mp_ecp_bird_hand_command));
	memcpy(tmp_string2, &mp_ecp_irp6_command, sizeof(mp_ecp_irp6_command));


	set_next_ecps_state(ecp_mp::task::ECP_GEN_BIRD_HAND, (int) 5, tmp_string1, sizeof(mp_ecp_bird_hand_command), 1, gripper_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, gripper_name.c_str(), gripper_name.c_str());

	set_next_ecps_state(ecp_mp::task::ECP_GEN_IRP6, (int) 5, tmp_string2, sizeof(mp_ecp_irp6_command), 1, manipulator_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, manipulator_name.c_str(), manipulator_name.c_str());

	//last IRp6 position from GraspI
	for (int i=0; i<6; ++i)
		mp_ecp_irp6_command.joint[i] = trgraspit->from_va.grasp_joint[i];
	//we want closed hand (desired_position from GraspIt), add delta_pos to apply forces
	mp_ecp_bird_hand_command.thumb_f[0].desired_position = 0.55 - trgraspit->from_va.grasp_joint[18] - delta_pos;
	mp_ecp_bird_hand_command.thumb_f[1].desired_position = 0.45 - trgraspit->from_va.grasp_joint[19] - delta_pos;
	mp_ecp_bird_hand_command.index_f[0].desired_position = 0.0;
	mp_ecp_bird_hand_command.index_f[1].desired_position = 0.55 - trgraspit->from_va.grasp_joint[16] - delta_pos;
	mp_ecp_bird_hand_command.index_f[2].desired_position = 0.45 - trgraspit->from_va.grasp_joint[17] - delta_pos;
	mp_ecp_bird_hand_command.ring_f[0].desired_position = 0.0;
	mp_ecp_bird_hand_command.ring_f[1].desired_position = 0.55 - trgraspit->from_va.grasp_joint[13] - delta_pos;
	mp_ecp_bird_hand_command.ring_f[2].desired_position = 0.45 - trgraspit->from_va.grasp_joint[14] - delta_pos;

	//check if adding delta_pos didn't cause out-of-boundary move
	for (int i=0; i<2; ++i)
		if (mp_ecp_bird_hand_command.thumb_f[i].desired_position < 0.0)
			mp_ecp_bird_hand_command.thumb_f[i].desired_position = 0.0;
	for (int i=0; i<3; ++i)
		if (mp_ecp_bird_hand_command.index_f[i].desired_position < 0.0)
			mp_ecp_bird_hand_command.index_f[i].desired_position = 0.0;
	for (int i=0; i<3; ++i)
		if (mp_ecp_bird_hand_command.ring_f[i].desired_position < 0.0)
			mp_ecp_bird_hand_command.ring_f[i].desired_position = 0.0;

	memcpy(tmp_string1, &mp_ecp_bird_hand_command, sizeof(mp_ecp_bird_hand_command));
	memcpy(tmp_string2, &mp_ecp_irp6_command, sizeof(mp_ecp_irp6_command));


	set_next_ecps_state(ecp_mp::task::ECP_GEN_IRP6, (int) 5, tmp_string2, sizeof(mp_ecp_irp6_command), 1, manipulator_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, manipulator_name.c_str(), manipulator_name.c_str());

	set_next_ecps_state(ecp_mp::task::ECP_GEN_BIRD_HAND, (int) 5, tmp_string1, sizeof(mp_ecp_bird_hand_command), 1, gripper_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, gripper_name.c_str(), gripper_name.c_str());


	//debugging
//	std::stringstream ss(std::stringstream::in | std::stringstream::out);
//	for (int i=12; i<20; ++i)
//		ss << "\n rec_val: " << trgraspit->from_va.grasp_joint[i];
//	sr_ecp_msg->message(ss.str().c_str());

	sr_ecp_msg->message("END GRASP");
}

task* return_created_mp_task(lib::configurator &_config)
{
	return new graspit(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
