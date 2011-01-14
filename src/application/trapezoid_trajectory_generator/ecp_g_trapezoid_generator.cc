/*
 * ecp_g_trapezoid_generator.cpp
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#include "ecp_g_trapezoid_generator.h"

namespace mrrocpp {

namespace ecp {

namespace trapezoid {

trapezoid_generator::trapezoid_generator(mrrocpp::ecp::common::task::task & ecp_task) :
	generator(ecp_task)
{
	// TODO Auto-generated constructor stub

}

trapezoid_generator::~trapezoid_generator()
{
	// TODO Auto-generated destructor stub
}

bool trapezoid_generator::first_step()
{
	return true;
}

bool trapezoid_generator::next_step()
{
	return true;
}

} // namespace trapezoid

}

}
