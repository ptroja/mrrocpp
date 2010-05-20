/*
 * ecp_t_conveyor_test.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_conveyor_test.h"
#include "ecp/conveyor/ecp_r_conv.h"

#include "../logger.h"

using namespace boost;
using namespace std;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace conveyor {

namespace task {

ecp_t_conveyor_test::ecp_t_conveyor_test(mrrocpp::lib::configurator& config) :
	task(config)
{
	logDbgEnabled = true;
	logEnabled = true;

	logDbg("ecp_t_conveyor_test::ecp_t_conveyor_test() 1\n");
	ecp_m_robot = new ecp::conveyor::robot(*this);

	logDbg("ecp_t_conveyor_test::ecp_t_conveyor_test() 2\n");

	sinus_gen = shared_ptr <ecp_g_conveyor_sinus> (new ecp_g_conveyor_sinus(*this, "[sinus_generator]"));
	logDbg("ecp_t_conveyor_test::ecp_t_conveyor_test() 3\n");
}

ecp_t_conveyor_test::~ecp_t_conveyor_test()
{
	delete ecp_m_robot;
}

void ecp_t_conveyor_test::main_task_algorithm(void)
{

	logDbg("ecp_t_conveyor_test::main_task_algorithm() 1\n");
	sinus_gen->Move();
	logDbg("ecp_t_conveyor_test::main_task_algorithm() 2\n");

	ecp_termination_notice();
}

} //namespace task

} //namespace conveyor

namespace common {

namespace task {

task* return_created_ecp_task(lib::configurator &config)
{
	return new mrrocpp::ecp::conveyor::task::ecp_t_conveyor_test(config);
}

} //namespace task

} //namespace common

} //namespace ecp

} //namespace mrrocpp
