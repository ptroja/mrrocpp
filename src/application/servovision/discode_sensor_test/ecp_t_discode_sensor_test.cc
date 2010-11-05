/*
 * ecp_t_discode_sensor_test.cc
 *
 *  Created on: Nov 4, 2010
 *      Author: mboryn
 */

#include "ecp_t_discode_sensor_test.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace discode_sensor_test {

namespace task {

ecp_t_discode_sensor_test::ecp_t_discode_sensor_test(mrrocpp::lib::configurator& config) : task(config)
{
	// TODO Auto-generated constructor stub
	ecp_m_robot = new ecp::irp6p_m::robot(*this);
	logger::log_dbg_enabled = logger::log_enabled = true;
}

ecp_t_discode_sensor_test::~ecp_t_discode_sensor_test()
{
	// TODO Auto-generated destructor stub
}

void ecp_t_discode_sensor_test::main_task_algorithm()
{
	sr_ecp_msg->message("ecp_t_discode_sensor_test::main_task_algorithm()");

	mrrocpp::ecp_mp::sensor::discode::discode_sensor ds(config, "dokladnie");
	ds.configure_sensor();

	mrrocpp::ecp::common::generator::ecp_g_discode_sensor_test g(*this, ds);

	g.Move();

	ecp_termination_notice();
}

}// namespace task {

} // namespace discode_sensor_test

namespace common {

namespace task {

task* return_created_ecp_task(lib::configurator &config)
{
	return new mrrocpp::ecp::discode_sensor_test::task::ecp_t_discode_sensor_test(config);
}

}// namespace task {

} // namespace common

}// namespace ecp

} // namespace mrrocpp
