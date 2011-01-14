/*
 * ecp_t_trapezoid_task.cpp
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#include "ecp_t_trapezoid_task.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

namespace mrrocpp {

namespace ecp {

namespace trapezoid {

trapezoid_task::trapezoid_task(lib::configurator &config) :
	task(config)
{
	ecp_m_robot = new ecp::irp6p_m::robot(*this);
	trapezoid_gen = boost::shared_ptr<trapezoid_generator>(new trapezoid_generator(*this));
	trapezoid_gen->set_params(0, 0.05, 0.05, 0.2, 2);
}

trapezoid_task::~trapezoid_task()
{
}

void trapezoid_task::main_task_algorithm()
{
	trapezoid_gen->Move();
	ecp_termination_notice();
}

} // namespace trapezoid

namespace common {
namespace task {
task* return_created_ecp_task(lib::configurator &config)
{
	return new mrrocpp::ecp::trapezoid::trapezoid_task(config);
}
} // namespace task
} // namespace common

} // namespace ecp
} // namespace mrrocpp
