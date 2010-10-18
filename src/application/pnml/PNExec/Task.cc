/*
 * Task.cc
 *
 *  Created on: Oct 4, 2009
 *      Author: ptroja
 */

#include <iostream>
#include <boost/foreach.hpp>

#include "Task.hh"
#include "../../rcsc/ecp_mp_t_fsautomat.h"

#include "base/lib/datastr.h"

namespace pnexec {

Task::Task(const task_t & task) {
	if (task.exec().present()) {
		type = EXEC;
//		const task_t::exec_type & e = task.exec().get();
//		std::cout << "task exec: " << e << std::endl;
	} else if (task.trajectory().present()) {
		const trajectory_t & trj = task.trajectory().get();
		std::cout << "task exec: " << trj.robot() << " @ " << trj.file() << std::endl;
		type = TRAJECTORY;
	} else {
		throw;
	}
}

Task::~Task(void) {
}

Task::TASK_TYPE Task::getType() const
{
	return type;
}

TrajectoryTask::TrajectoryTask(const task_t & task) : Task(task),
	robot(task.trajectory().get().robot()),
	file(task.trajectory().get().file()),
	robot_name(lib::returnProperRobot(robot))
{
}

// TODO: it probably should be:
//ecp_next_state_t TrajectoryTask::execute(void)
//! \return {returns true if MP has to wait for task completion signal}
mrrocpp::lib::robot_name_t TrajectoryTask::execute(mrrocpp::mp::common::robots_t & _robots, workers_t & _workers)
{
	bool robot_name_found = false;
	BOOST_FOREACH(mrrocpp::mp::common::robot_pair_t & robot_node, _robots) {
		if (robot_node.second->robot_name == robot_name) {
			robot_name_found = true;
			break;
		}
	}
	assert(robot_name_found);

	mrrocpp::mp::robot::robot & _robot = *_robots[robot_name];

	std::cerr << "executing for " << lib::toString(_robot.robot_name) << std::endl; fflush(stderr);

	_robot.mp_command.command = lib::NEXT_STATE;
	_robot.mp_command.ecp_next_state.mp_2_ecp_next_state = mrrocpp::ecp_mp::generator::ECP_GEN_SMOOTH;

	strncpy(_robot.mp_command.ecp_next_state.mp_2_ecp_next_state_string,
			file.c_str(),
			sizeof(_robot.mp_command.ecp_next_state.mp_2_ecp_next_state_string));

	_robot.mp_command.command = lib::NEXT_STATE;
	_robot.communicate = true;

	std::cerr << "commanding " << robot << " @ " << file << std::endl;

	return robot_name;
}

} // namespace
