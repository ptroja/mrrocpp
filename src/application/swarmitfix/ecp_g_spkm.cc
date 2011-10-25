/*
 * Author: Piotr Trojanek
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

void spkm_pose::request_segment_execution(robot_t & robot, const lib::spkm::segment_t & segment)
{
	// Copy the motion type
	robot.epos_external_command_data_port.data.motion_variant = segment.motion_type;

	// Translate the Homog_matrix pose to POD XYZ-euler-ZYZ representation
	lib::Xyz_Euler_Zyz_vector tmp;
	segment.goal_pose.get_xyz_euler_zyz(tmp);
	tmp.to_table(robot.epos_external_command_data_port.data.desired_position);

	// Mark the current buffer as active
	robot.epos_external_command_data_port.set();
}

bool spkm_pose::first_step()
{
	sr_ecp_msg.message("spkm_pose: first_step");

	// skip the empty command sequence
	if (ecp_t.mp_command.ecp_next_state.spkm_segment_sequence.empty())
		return false;

	// set iterator to the first command
	segment_iterator = ecp_t.mp_command.ecp_next_state.spkm_segment_sequence.begin();

	// Prepare command for execution of the first motion segment
	request_segment_execution(*the_robot, *segment_iterator);

	return true;
}

bool spkm_pose::next_step()
{
	sr_ecp_msg.message("spkm_pose: next_step");

	// A co to jest??? (ptroja)
	if (the_robot->epos_motor_reply_data_request_port.get() == mrrocpp::lib::NewData) {

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: " << the_robot->epos_motor_reply_data_request_port.data.epos_controller[3].position;

		sr_ecp_msg.message(ss.str());
	}

	// Check motion status of the PKM
	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (the_robot->epos_motor_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	// Check if the commanded motion is already completed
	if (motion_in_progress) {
		// Request new status data
		the_robot->epos_motor_reply_data_request_port.set_request();
		return true;
	}

	// Increment the segment iterator
	++segment_iterator;

	// Check if the motion sequence is completed
	if (segment_iterator == ecp_t.mp_command.ecp_next_state.spkm_segment_sequence.end())
		return false;

	// Prepare command for execution of a next motion segment
	request_segment_execution(*the_robot, *segment_iterator);

	// Continue
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

