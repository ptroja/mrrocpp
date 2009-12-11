/*
 * ecp_t_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_t_mboryn.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace task {

ecp_t_mboryn::ecp_t_mboryn(mrrocpp::lib::configurator& _configurator)
: task(_configurator)
{
	ecp_m_robot = new ecp::irp6ot::robot(*this);
	ecp_g_mboryn_ = new generator::ecp_g_mboryn(*this);

	sr_ecp_msg->message("ecp_t_mboryn::ecp_t_mboryn() finished.");
}

ecp_t_mboryn::~ecp_t_mboryn() {
	// TODO Auto-generated destructor stub
}

void ecp_t_mboryn::main_task_algorithm(void)
{
	ecp_g_mboryn_->Move();
	ecp_termination_notice();
}

} // namespace task

} // namespace irp6ot

namespace common {

namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::ecp_t_mboryn(_config);
}

} // namespace task

} // namespace common


} // namespace ecp

} // namespace mrrocpp
