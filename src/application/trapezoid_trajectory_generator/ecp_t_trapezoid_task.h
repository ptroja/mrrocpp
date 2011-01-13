/*
 * ecp_t_trapezoid_task.h
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#ifndef ECP_T_TRAPEZOID_TASK_H_
#define ECP_T_TRAPEZOID_TASK_H_

#include "ecp_task.h"

namespace mrrocpp {

namespace ecp {

namespace trapezoid {

class trapezoid_task : public mrrocpp::ecp::common::task::task
{
public:
	trapezoid_task();
	virtual ~trapezoid_task();
};

}

}

}

#endif /* ECP_T_TRAPEZOID_TASK_H_ */
