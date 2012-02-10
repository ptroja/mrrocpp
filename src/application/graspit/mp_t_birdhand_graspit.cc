#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "base/mp/mp_task.h"

#include "base/lib/mrmath/mrmath.h"
#include "robot/bird_hand/dp_bird_hand.h"
#include "mp_t_birdhand_graspit.h"
#include "ecp_mp_t_graspit.h"
#include "ecp_mp_g_birdhand.h"
#include "robot/bird_hand/const_bird_hand.h"
#include <iostream>
#include <string>
#include <sstream>
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"

namespace mrrocpp {
namespace mp {
namespace task {

graspit::graspit(lib::configurator &_config) :
		task(_config)
{
	trgraspit =
			new ecp_mp::transmitter::TRGraspit(ecp_mp::transmitter::TRANSMITTER_GRASPIT, "[transmitter_graspit]", *this);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void graspit::create_robots()
{
	ACTIVATE_MP_ROBOT(conveyor);

	ACTIVATE_MP_ROBOT(bird_hand);

	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
}

void graspit::main_task_algorithm(void)
{

	//delta for hand position control
	const double delta_pos = 0.1;

	sr_ecp_msg->message("START GRASP");

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	int port = config.value <int>("graspit_port", "[transmitter_graspit]");
	std::string node_name = config.value <std::string>("graspit_node_name", "[transmitter_graspit]");

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
	if (config.value <int>("is_irp6ot_m_active", lib::UI_SECTION)) {
		manipulator_name = lib::irp6ot_m::ROBOT_NAME;
		if (config.value <int>("is_bird_hand_active", lib::UI_SECTION)) {
			gripper_name = lib::bird_hand::ROBOT_NAME;
		} else {
			// TODO: throw
		}
	} else if (config.value <int>("is_irp6p_m_active", lib::UI_SECTION)) {
		manipulator_name = lib::irp6p_m::ROBOT_NAME;
		if (config.value <int>("is_bird_hand_active", lib::UI_SECTION)) {
			gripper_name = lib::bird_hand::ROBOT_NAME;
		} else {
			// TODO: throw
		}
	} else {
		// TODO: throw
	}

	char tmp_string1[lib::MP_2_ECP_SERIALIZED_DATA_SIZE];
	char tmp_string2[lib::MP_2_ECP_SERIALIZED_DATA_SIZE];

	struct _irp6
	{
		double joint[6];
	} mp_ecp_irp6_command;
	lib::bird_hand::command mp_ecp_bird_hand_command;

	//middle IRp6 position from GraspIt
	for (int i = 0; i < 6; ++i)
		mp_ecp_irp6_command.joint[i] = trgraspit->from_va.grasp_joint[i + 6];
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

	set_next_ecp_state(ecp_mp::task::ECP_GEN_BIRD_HAND, (int) 5, tmp_string1, gripper_name);

	wait_for_task_termination(false, gripper_name);

	set_next_ecp_state(ecp_mp::task::ECP_GEN_IRP6, (int) 5, tmp_string2, manipulator_name);

	wait_for_task_termination(false, manipulator_name);

	//last IRp6 position from GraspI
	for (int i = 0; i < 6; ++i)
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
	for (int i = 0; i < 2; ++i)
		if (mp_ecp_bird_hand_command.thumb_f[i].desired_position < 0.0)
			mp_ecp_bird_hand_command.thumb_f[i].desired_position = 0.0;
	for (int i = 0; i < 3; ++i)
		if (mp_ecp_bird_hand_command.index_f[i].desired_position < 0.0)
			mp_ecp_bird_hand_command.index_f[i].desired_position = 0.0;
	for (int i = 0; i < 3; ++i)
		if (mp_ecp_bird_hand_command.ring_f[i].desired_position < 0.0)
			mp_ecp_bird_hand_command.ring_f[i].desired_position = 0.0;

	memcpy(tmp_string1, &mp_ecp_bird_hand_command, sizeof(mp_ecp_bird_hand_command));
	memcpy(tmp_string2, &mp_ecp_irp6_command, sizeof(mp_ecp_irp6_command));

	set_next_ecp_state(ecp_mp::task::ECP_GEN_IRP6, (int) 5, tmp_string2, manipulator_name);

	wait_for_task_termination(false, manipulator_name);

	set_next_ecp_state(ecp_mp::task::ECP_GEN_BIRD_HAND, (int) 5, tmp_string1, gripper_name);

	wait_for_task_termination(false, gripper_name);

	//debugging
	//	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	//	for (int i = 12; i < 20; ++i)
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
