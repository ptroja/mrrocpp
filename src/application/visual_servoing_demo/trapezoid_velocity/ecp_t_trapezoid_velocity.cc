/*
 * trapezoid_velocity_task.cc
 *
 *  Created on: 25-02-2011
 *      Author: mateusz
 */

#include "ecp_t_trapezoid_velocity.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

trapezoid_velocity_task::trapezoid_velocity_task(lib::configurator &config) :
	mrrocpp::ecp::common::task::task(config)
{
	ecp_m_robot = boost::shared_ptr<robot_t>(new ecp::irp6p_m::robot(*this));
	trapezoid_gen = boost::shared_ptr<generator::trapezoid_velocity>(new generator::trapezoid_velocity(*this));
	trapezoid_gen->set_params(0, 0.05, 0.05, 0.2);
}

trapezoid_velocity_task::~trapezoid_velocity_task()
{
	// TODO Auto-generated destructor stub
}

void trapezoid_velocity_task::main_task_algorithm()
{
	sr_ecp_msg->message("Starting motion...");
	trapezoid_gen->Move();
	sr_ecp_msg->message("Motion finished.");
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new trapezoid_velocity_task(config);
}

} // namespace task

} // namespace common

} // namespace ecp

} // namespace mrrocpp
