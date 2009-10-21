/*
 * Task.cc
 *
 *  Created on: Oct 4, 2009
 *      Author: ptroja
 */

#include "Task.hh"

#include <iostream>

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

} // namespace
