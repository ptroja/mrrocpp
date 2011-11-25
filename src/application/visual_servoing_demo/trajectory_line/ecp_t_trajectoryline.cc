/*
 * ecp_t_trajectoryline.cc
 *
 *  Created on: 11-08-2011
 *      Author: mateusz
 */

#include "ecp_t_trajectoryline.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_t_trajectory_line::ecp_t_trajectory_line(mrrocpp::lib::configurator& config) :
		mrrocpp::ecp::common::task::task(config)
{
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);

	gen = boost::shared_ptr<ecp_g_trajectory_line>(new ecp_g_trajectory_line(*this, "[trajectory_line]"));
}

ecp_t_trajectory_line::~ecp_t_trajectory_line()
{
	// TODO Auto-generated destructor stub
}

void ecp_t_trajectory_line::main_task_algorithm()
{
	gen->Move();
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_trajectory_line(config);
}

} /* namespace task */
} /* namespace common */
} /* namespace ecp */
} /* namespace mrrocpp */
