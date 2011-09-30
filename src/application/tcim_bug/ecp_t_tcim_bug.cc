/*
 * ecp_t_tcim_bug.cc
 *
 *  Created on: 07-06-2011
 *      Author: mboryn
 */

#include "ecp_t_tcim_bug.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {


ecp_t_tcim_bug::ecp_t_tcim_bug(mrrocpp::lib::configurator& config) : mrrocpp::ecp::common::task::task(config)
{
	ecp_m_robot = boost::shared_ptr<robot_t>(new ecp::irp6p_m::robot(*this));


	gen = boost::shared_ptr<ecp_g_tcim_bug> (new ecp_g_tcim_bug(*this));
}

ecp_t_tcim_bug::~ecp_t_tcim_bug()
{
}

void ecp_t_tcim_bug::main_task_algorithm()
{
	sr_ecp_msg->message("ecp_t_tcim_bug 1");

	gen->Move();

	sr_ecp_msg->message("ecp_t_tcim_bug 2");

	termination_notice();
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_tcim_bug(config);
}

}

}

}

}
