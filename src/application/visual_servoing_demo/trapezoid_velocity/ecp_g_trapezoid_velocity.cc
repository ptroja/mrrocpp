/*
 * trapezoid_velocity.cc
 *
 *  Created on: 25-02-2011
 *      Author: mateusz
 */

#include "ecp_g_trapezoid_velocity.h"

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

trapezoid_velocity_gen::trapezoid_velocity_gen(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name):
		common::generator::generator(ecp_task)
{
}

trapezoid_velocity_gen::~trapezoid_velocity_gen()
{
	// TODO Auto-generated destructor stub
}

bool trapezoid_velocity_gen::first_step()
{
	return true;
}

bool trapezoid_velocity_gen::next_step()
{
	return true;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp
