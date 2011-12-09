/*
 * Author: Piotr Trojanek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

//
//
//
// smb_action
//
//
//

//constructor with parameters: task and time to sleep [s]
smb_action::smb_action(task_t & _ecp_task, const lib::smb::next_state_t::action_sequence_t & _actions) :
		generator_t(_ecp_task),
		actions(_actions)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

void smb_action::request_action_execution(robot_t & robot, const lib::smb::action & action)
{
	// Copy the motion duration
	robot.epos_external_command_data_port.data.estimated_time = action.getDuration();

	// Setup motion commands
	// TODO: this have to be UP-rotate-DOWN sequence
	if(action.getRotationPin() != 0) {
		// Copy rotation command
		robot.epos_external_command_data_port.data.desired_position[0] = action.getdTheta();

		// Trigger command execution
		robot.epos_external_command_data_port.set();
	}

	if(action.getdPkmTheta()) {
		// Copy rotation command
		robot.epos_external_command_data_port.data.desired_position[1] = action.getdTheta();

		// Trigger command execution
		robot.epos_external_command_data_port.set();
	}
}

bool smb_action::first_step()
{
	sr_ecp_msg.message("smb_action: first_step");

	std::cerr << "ECP # of actions = " << actions.size() << std::endl;
	for(lib::smb::next_state_t::action_sequence_t::const_iterator it = actions.begin();
			it != actions.end();
			++it) {
		std::cerr << "rotation pin\n" << it->getRotationPin() << std::endl;
		if(it->getRotationPin()) std::cerr << "dTheta " << it->getdTheta() << std::endl;
		std::cerr << "dPkmTheta" << it->getdPkmTheta() << std::endl;
	}

	// skip the empty command sequence
	if(actions.empty())
		return false;

	// set iterator to the first command
	action_iterator = actions.begin();

	// Prepare command for execution of the first motion action
	request_action_execution(*the_robot, *action_iterator);

	// Request status report
	the_robot->epos_external_reply_data_request_port.set_request();

	return true;
}

bool smb_action::next_step()
{
	sr_ecp_msg.message("smb_action: next_step");

	// A co to jest??? (ptroja)
	//if (the_robot->epos_motor_reply_data_request_port.get() == mrrocpp::lib::NewData) {
	//}

	// waits 20ms to check epos state
	delay(20);
	the_robot->epos_external_reply_data_request_port.get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
		if (the_robot->epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		// Request status report
		the_robot->epos_external_reply_data_request_port.set_request();
		return true;
	}

	// Increment action iterator
	++action_iterator;

	// Check if the motion sequence is completed
	if (action_iterator == actions.end())
		return false;

	// Prepare command for execution of a next motion action
	request_action_execution(*the_robot, *action_iterator);

	// Request status report
	the_robot->epos_external_reply_data_request_port.set_request();

	// Continue
	return true;
}

//
//
//
// smb_quickstop
//
//
//

smb_quickstop::smb_quickstop(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

bool smb_quickstop::first_step()
{
	//the_robot->epos_brake_command_data_port.data = true;
	the_robot->epos_brake_command_data_port.set();

	return true;
}

bool smb_quickstop::next_step()
{
	return true;
}

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp
