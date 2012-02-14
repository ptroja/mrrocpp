/*
 * ecp_g_conveyor_sinus.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */
#include <cmath>
#include <stdexcept>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_conveyor_uniform_acceleration.h"
#include "base/lib/logger.h"
#include "base/lib/configurator.h"

using namespace std;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ecp_g_conveyor_uniform_acceleration::ecp_g_conveyor_uniform_acceleration(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
	common::generator::generator(ecp_task)
{
	motion_steps = 30;
	dt = motion_steps * 0.002;
	acceleration = ecp_task.config.value <double> ("acceleration", section_name);
	max_speed = ecp_task.config.value <double> ("max_speed", section_name);

	if (!(max_speed * acceleration > 0)) {
		throw runtime_error("ecp_g_conveyor_uniform_acceleration: !(max_speed * acceleration > 0)");
	}

	string log_enabled_name = "vs_log_enabled";
	if (ecp_task.config.exists(log_enabled_name, section_name)
			&& ecp_task.config.value <bool> (log_enabled_name, section_name)) {
		unsigned int capacity = ecp_task.config.value <unsigned int> ("vs_log_capacity", section_name);
		std::string server_addr = ecp_task.config.value <std::string> ("vs_log_server_addr", section_name);
		int server_port = ecp_task.config.value <int> ("vs_log_server_port", section_name);

		log_client = boost::shared_ptr <logger_client>(new logger_client(capacity, server_addr, server_port, "conveyor_position;"));
	}


	t = 0;
	current_speed = 0;
}

ecp_g_conveyor_uniform_acceleration::~ecp_g_conveyor_uniform_acceleration()
{
}

bool ecp_g_conveyor_uniform_acceleration::first_step()
{
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.get_arm_type = lib::JOINT;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::JOINT;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = motion_steps;
	the_robot->ecp_command.value_in_step_no = motion_steps - 3;
	//the_robot->ecp_command.robot_model.type = lib::ARM_KINEMATIC_MODEL;

	//	for (int i = 0; i < 6; i++) {
	the_robot->ecp_command.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;
	//	}

	initial_position_saved = false;
	t = 0;
	current_speed = 0;

//	log_dbg("bool ecp_g_conveyor_uniform_acceleration::first_step()\n");

	return true;
}
bool ecp_g_conveyor_uniform_acceleration::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	if (!initial_position_saved) {
		current_position = the_robot->reply_package.arm.pf_def.joint_coordinates[0];
		initial_position_saved = true;
	}

	double dv = acceleration * dt;

	if (fabs(current_speed) < max_speed) {
		current_speed = current_speed + dv;
	} else {
		current_speed = max_speed;
	}

	double ds = current_speed * dt;

	current_position = current_position + ds;

	the_robot->ecp_command.arm.pf_def.arm_coordinates[0] = current_position;

	sprintf(msg.text, "%g;",
			current_position
	);

	if (log_client.get() != NULL) {
		log_client->log(msg);
	}

	t += dt;
	return true;
}

}//namespace

}

}

}
