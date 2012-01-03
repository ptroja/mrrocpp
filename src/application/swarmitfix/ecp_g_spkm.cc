/*
 * Author: Piotr Trojanek
 */

#include "base/lib/periodic_timer.h"

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

spkm_pose::spkm_pose(task_t & _ecp_task, const lib::spkm::next_state_t::segment_sequence_t & _segments) :
		generator_t(_ecp_task),
		segments(_segments),
		wakeup(20)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

void spkm_pose::request_segment_execution(robot_t & robot, const lib::spkm::segment_t & segment)
{
	// Copy the motion type
	robot.epos_external_command_data_port.data.pose_specification = lib::spkm::TOOL_XYZ_EULER_ZYZ;
	robot.epos_external_command_data_port.data.motion_variant = segment.motion_type;
	robot.epos_external_command_data_port.data.estimated_time = segment.duration;

	// Translate the Homog_matrix pose to POD XYZ-euler-ZYZ representation
	lib::Xyz_Euler_Zyz_vector tmp;
	segment.goal_pose.get_xyz_euler_zyz(tmp);
	tmp.to_table(robot.epos_external_command_data_port.data.desired_position);

	std::cerr << "ECP receives\n" << segment.goal_pose << std::endl;

	// Mark the current buffer as active
	robot.epos_external_command_data_port.set();
}

bool spkm_pose::first_step()
{
	sr_ecp_msg.message("spkm_pose: first_step");

	std::cerr << "ECP # of segments = " << segments.size() << std::endl;
	for(lib::spkm::next_state_t::segment_sequence_t::const_iterator it = segments.begin();
			it != segments.end();
			++it) {
		std::cerr << "pose\n" << it->goal_pose << std::endl;
		std::cerr << "motion type " << it->motion_type << std::endl;
		std::cerr << "duration " << it->duration << std::endl;
		std::cerr << "guarded_motion " << it->guarded_motion << std::endl;
	}

	// skip the empty command sequence
	if(segments.empty())
		return false;

	// set iterator to the first command
	segment_iterator = segments.begin();

	// Prepare command for execution of the first motion segment
	request_segment_execution(*the_robot, *segment_iterator);

	// Request status report
	the_robot->epos_external_reply_data_request_port.set_data = lib::spkm::TOOL_XYZ_EULER_ZYZ;
	the_robot->epos_external_reply_data_request_port.set_request();

	return true;
}

bool spkm_pose::next_step()
{
	sr_ecp_msg.message("spkm_pose: next_step");

	// A co to jest??? (ptroja)
	//if (the_robot->epos_motor_reply_data_request_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
	//}

	the_robot->epos_external_reply_data_request_port.get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		if (the_robot->epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		// Request status report
		the_robot->epos_external_reply_data_request_port.set_request();

		// Delay until next EPOS query
		wakeup.sleep();

		return true;
	}

	// Increment segment iterator
	++segment_iterator;

	// Check if the motion sequence is completed
	if (segment_iterator == segments.end())
		return false;

	// Prepare command for execution of a next motion segment
	request_segment_execution(*the_robot, *segment_iterator);

	// Request status report
	the_robot->epos_external_reply_data_request_port.set_request();

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
	//the_robot->epos_brake_command_data_port.data = true;
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
