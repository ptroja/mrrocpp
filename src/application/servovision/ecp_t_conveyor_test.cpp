/*
 * ecp_t_conveyor_test.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_conveyor_test.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_t_conveyor_test::ecp_t_conveyor_test(mrrocpp::lib::configurator& configurator):task(config)
{
	ecp_m_robot = new ecp::irp6ot::robot(*this);
}

ecp_t_conveyor_test::~ecp_t_conveyor_test()
{
	delete ecp_m_robot;
}

void main_task_algorithm(void)
{

	ecp_termination_notice();
}

task* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_conveyor_test(config);
}

}//namespace

}

}

}
