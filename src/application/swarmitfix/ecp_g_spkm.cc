/*
 * generator/ecp_g_epos.cc
 *
 *Author: yoyek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

//
//
//
// spkm_pose
//
//
//


//constructor with parameters: task and time to sleep [s]
spkm_pose::spkm_pose(task_t & _ecp_task) :
	generator_t(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

void spkm_pose::create_ecp_mp_reply()
{

}

void spkm_pose::get_mp_ecp_command()
{
	//	memcpy(&mp_ecp_epos_gen_parameters_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_epos_gen_parameters_structure));

	//	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool spkm_pose::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	sr_ecp_msg.message("epos first_step");

	// Fill the goal_pos data
	the_robot->epos_joint_command_data_port.data.desired_position[0] = 0;
	//the_robot->epos_joint_command_data_port.data.desired_position[1] = ...;

	// Mark the current buffer as used
	the_robot->epos_joint_command_data_port.set();

	return true;
}

bool spkm_pose::next_step()
{
	sr_ecp_msg.message("epos next_step");
#if 0
	if (epos_reply_data_request_port->get() == mrrocpp::lib::NewData) {

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: " << epos_reply_data_request_port->data.epos_controller[3].position;

		sr_ecp_msg.message(ss.str().c_str());

	}

	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (epos_reply_data_request_port->data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		epos_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
#endif

	return true;
}

//
//
//
// spkm_quickstop
//
//
//


spkm_quickstop::spkm_quickstop(task_t & _ecp_task) :
	generator_t(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

void spkm_quickstop::create_ecp_mp_reply()
{

}

void spkm_quickstop::get_mp_ecp_command()
{
	//	memcpy(&mp_ecp_epos_gen_parameters_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_epos_gen_parameters_structure));

	//	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool spkm_quickstop::first_step()
{
	the_robot->epos_brake_command_data_port.data = true;
	the_robot->epos_brake_command_data_port.set();

	return true;
}

bool spkm_quickstop::next_step()
{

	return true;

}

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

