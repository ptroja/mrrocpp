/*
 * ecp_t_conveyor_test.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_CONVEYOR_TEST_H_
#define ECP_T_CONVEYOR_TEST_H_

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

class ecp_t_conveyor_test : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_conveyor_test(mrrocpp::lib::configurator& configurator);
	virtual ~ecp_t_conveyor_test();
	void main_task_algorithm(void);
};

} //namespace

}

}

}

#endif /* ECP_T_CONVEYOR_TEST_H_ */
