/*
 * ecp_t_discode_sensor_test.cc
 *
 *  Created on: Nov 4, 2010
 *      Author: mboryn
 */

#include <string>
#include "ecp_t_discode_sensor_test.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace discode_sensor_test {

namespace task {

using namespace std;

ecp_t_discode_sensor_test::ecp_t_discode_sensor_test(mrrocpp::lib::configurator& config) :
	task(config)
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

	mrrocpp::ecp_mp::sensor::discode::discode_sensor ds(config, "[dokladnie]");

	logger::log_dbg("ecp_t_discode_sensor_test::main_task_algorithm(): before ds.configure_sensor()\n");
	ds.configure_sensor();
	logger::log_dbg("ecp_t_discode_sensor_test::main_task_algorithm(): after ds.configure_sensor()\n");

	while (1) {
		string received = ds.call_remote_procedure<string>(string("void ecp_t_discode_sensor_test::main_task_algorithm()"));
		logger::log_dbg("ecp_t_discode_sensor_test::main_task_algorithm(): received: \"%s\"\n", received.c_str());
	}

	mrrocpp::ecp::common::generator::ecp_g_discode_sensor_test g(*this, &ds);
	logger::log_dbg("ecp_t_discode_sensor_test::main_task_algorithm(): 4\n");
	g.Move();
	logger::log_dbg("ecp_t_discode_sensor_test::main_task_algorithm(): 5\n");
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
