/*
 * Task.cc
 *
 *  Created on: Oct 4, 2009
 *      Author: ptroja
 */

#include <iostream>

#include "Task.hh"
#include "ecp_mp/ecp_mp_t_fsautomat.h"

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
void TrajectoryTask::execute(mrrocpp::mp::common::robots_t & _robots)
{
	_robots[robot_name]->ecp_td.ecp_next_state.mp_2_ecp_next_state =
		mrrocpp::ecp_mp::task::ECP_GEN_SMOOTH;

	strncpy(_robots[robot_name]->ecp_td.ecp_next_state.mp_2_ecp_next_state_string,
			file.c_str(),
			sizeof(_robots[robot_name]->ecp_td.ecp_next_state.mp_2_ecp_next_state_string));


	_robots[robot_name]->ecp_td.mp_command = lib::NEXT_STATE;
	_robots[robot_name]->communicate = true;

	std::cout << "commanding " << robot << " @ " << file << std::endl;
}

} // namespace
