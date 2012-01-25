/*
 * ecp_t_conveyor_test.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_conveyor_test.h"
#include "robot/conveyor/ecp_r_conv.h"

#include "base/lib/logger.h"

#include "../ecp_mp_g_visual_servo_tester.h"

using namespace boost;
using namespace std;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace conveyor {

namespace task {

ecp_t_conveyor_test::ecp_t_conveyor_test(mrrocpp::lib::configurator& config) :
	mrrocpp::ecp::common::task::task(config)
{
	log_dbg_enabled = true;
	log_enabled = true;

	log_dbg("ecp_t_conveyor_test::ecp_t_conveyor_test() 1\n");
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::conveyor::robot(*this);

	log_dbg("ecp_t_conveyor_test::ecp_t_conveyor_test() 2\n");

	sinus_gen = boost::shared_ptr <ecp_g_conveyor_sinus> (new ecp_g_conveyor_sinus(*this, "[sinus_generator]"));
	log_dbg("ecp_t_conveyor_test::ecp_t_conveyor_test() 3\n");
}

void ecp_t_conveyor_test::main_task_algorithm(void)
{

	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_CONVEYOR_VS_TEST) {
			sinus_gen->Move();
		} else {
			log("ecp_t_conveyor_test::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
	}

	termination_notice();
}

} //namespace task

} //namespace conveyor

namespace common {

namespace task {

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new mrrocpp::ecp::conveyor::task::ecp_t_conveyor_test(config);
}

} //namespace task

} //namespace common

} //namespace ecp

} //namespace mrrocpp
