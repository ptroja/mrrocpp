/*
 * Task.hh
 *
 *  Created on: Oct 4, 2009
 *      Author: ptroja
 */

#ifndef TASK_HH_
#define TASK_HH_

#include "pipepnml.hxx"

#include <string>
#include <iostream>

#include "lib/impconst.h"
#include "mp/mp_generator.h"

namespace pnexec {

class Task {
	public:
		Task(const task_t & task);

		virtual ~Task();

		typedef enum _TASK_TYPE {
			EXEC,
			TRAJECTORY
		} TASK_TYPE;

		TASK_TYPE getType() const;

		virtual void execute(mrrocpp::mp::common::robots_t & _robots) = 0;

	private:
		TASK_TYPE type;

		mrrocpp::lib::ROBOT_ENUM robot_name;
};

class ExecTask : public Task {
	private:
		const std::string job;

	public:
		ExecTask(const task_t & task) : Task(task), job(task.exec().get()) {
			std::cout << "ExecTask()" << std::endl;
		}

		virtual void execute(mrrocpp::mp::common::robots_t & _robots) {
			system(job.c_str());
		}

		virtual ~ExecTask() {
			std::cout << "~ExecTask()" << std::endl;
		}
};

class TrajectoryTask : public Task {
	private:
		const std::string robot;
		const std::string file;

	public:
		TrajectoryTask(const task_t & task)
		: Task(task),
		robot(task.trajectory().get().robot()),
		file(task.trajectory().get().file()) {
		}

		virtual void execute(mrrocpp::mp::common::robots_t & _robots) {
			std::cout << robot << " @ " << file << std::endl;
		}
};

} // namespace

#endif /* TASK_HH_ */
