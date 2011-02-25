/*
 * trapezoid_velocity_task.cc
 *
 *  Created on: 25-02-2011
 *      Author: mateusz
 */

#include "ecp_t_trapezoid_velocity.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

trapezoid_velocity_task::trapezoid_velocity_task(lib::configurator &config) :
		mrrocpp::ecp::common::task::task(config)
{
	// TODO Auto-generated constructor stub

}

trapezoid_velocity_task::~trapezoid_velocity_task()
{
	// TODO Auto-generated destructor stub
}

void trapezoid_velocity_task::main_task_algorithm()
{

}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new trapezoid_velocity_task(config);
}

} // namespace task

} // namespace common

} // namespace ecp

} // namespace mrrocpp
