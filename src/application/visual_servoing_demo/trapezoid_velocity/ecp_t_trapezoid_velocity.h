/*
 * trapezoid_velocity_task.h
 *
 *  Created on: 25-02-2011
 *      Author: mateusz
 */

#ifndef TRAPEZOID_VELOCITY_TASK_H_
#define TRAPEZOID_VELOCITY_TASK_H_

#include "base/ecp/ecp_task.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

class trapezoid_velocity_task: public mrrocpp::ecp::common::task::task {
public:
	trapezoid_velocity_task(lib::configurator &config);
	virtual ~trapezoid_velocity_task();

	void main_task_algorithm();
};

}

}

}

}

#endif /* TRAPEZOID_VELOCITY_TASK_H_ */
