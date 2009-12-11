/*
 * ecp_g_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_g_mboryn.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

ecp_g_mboryn::ecp_g_mboryn(common::task::task & _ecp_task)
: generator(_ecp_task)
{
}

ecp_g_mboryn::~ecp_g_mboryn() {
}

bool ecp_g_mboryn::first_step()
{
	return false;
}

bool ecp_g_mboryn::next_step()
{
	return false;
}

} // namespace mrrocpp

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp
