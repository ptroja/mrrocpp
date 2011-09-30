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
#include <map>

#include "Worker.hh"

#include "mp/mp.h"
#include "lib/datastr.h"

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

		virtual mrrocpp::lib::robot_name_t execute(mrrocpp::mp::common::robots_t & _robots, workers_t & _workers) = 0;

	private:
		TASK_TYPE type;

		mrrocpp::lib::robot_name_t robot_name;
};

class ExecTask : public Task {
	private:
		const std::string job;

	public:
		ExecTask(const task_t & task) : Task(task), job(task.exec().get()) {
			std::cout << "ExecTask()" << std::endl;
		}

		virtual mrrocpp::lib::robot_name_t execute(mrrocpp::mp::common::robots_t & _robots, workers_t & _workers) {
//			system(job.c_str());
			std::cerr << "executing " << job << std::endl;
			return mrrocpp::lib::ROBOT_UNDEFINED;
		}

		virtual ~ExecTask() {
			std::cout << "~ExecTask()" << std::endl;
		}
};

class TrajectoryTask : public Task {
	private:
		const std::string robot;
		const std::string file;

		const mrrocpp::lib::robot_name_t robot_name;

	public:
		TrajectoryTask(const task_t & task);

		virtual mrrocpp::lib::robot_name_t execute(mrrocpp::mp::common::robots_t & _robots, workers_t & _workers);
};

} // namespace

#endif /* TASK_HH_ */
