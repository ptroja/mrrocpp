/*
 * generator/ecp_g_shead.cc
 *
 *Author: yoyek
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time/time_duration.hpp>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

rotate::rotate(task_t & _ecp_task, const double & _goal) :
	generator_t(_ecp_task),
	goal(_goal),
	query_interval(boost::posix_time::milliseconds(20))
{
}

bool rotate::first_step()
{
	// Parameters copying
	the_robot->epos_joint_command_data_port.data.desired_position[0] = goal;

	// Request status data
	the_robot->epos_joint_reply_data_request_port.set_request();

	// Record
	wakeup = boost::get_system_time();

	return true;
}

bool rotate::next_step()
{
	the_robot->epos_joint_reply_data_request_port.get();

	if(the_robot->epos_joint_reply_data_request_port.data.epos_controller[0].motion_in_progress) {
		// Request next status data
		the_robot->epos_joint_reply_data_request_port.set_request();

		// Sleep before next query
		wakeup += query_interval;
		boost::thread::sleep(wakeup);

		return true;
	}

	return false;
}

control::control(task_t & _ecp_task, const lib::shead::next_state::control_t & _control) :
	generator_t(_ecp_task),
	control_state(_control)
{
}

bool control::first_step()
{
	// Parameters copying
	the_robot->shead_head_soldification_data_port.data = control_state.solidify;
	the_robot->shead_vacuum_activation_data_port.data = control_state.vacuum;

	return true;
}

bool control::next_step()
{
	// This is a single-step generator
	return false;
}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

