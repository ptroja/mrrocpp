/*
 * ecp_t_conveyor_uniform_acceleration.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_conveyor_uniform_acceleration.h"
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

ecp_t_conveyor_uniform_acceleration::ecp_t_conveyor_uniform_acceleration(mrrocpp::lib::configurator& config) :
	mrrocpp::ecp::common::task::task(config)
{
	log_dbg_enabled = true;
	log_enabled = true;

	log_dbg("ecp_t_conveyor_uniform_acceleration::ecp_t_conveyor_uniform_acceleration() 1\n");
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::conveyor::robot(*this);

	log_dbg("ecp_t_conveyor_uniform_acceleration::ecp_t_conveyor_uniform_acceleration() 2\n");

	uniform_acceleration_gen
			= boost::shared_ptr <ecp_g_conveyor_uniform_acceleration> (new ecp_g_conveyor_uniform_acceleration(*this, "[uniform_acceleration_generator]"));
	log_dbg("ecp_t_conveyor_uniform_acceleration::ecp_t_conveyor_uniform_acceleration() 3\n");
}

void ecp_t_conveyor_uniform_acceleration::main_task_algorithm(void)
{
	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_CONVEYOR_VS_TEST) {
			if(uniform_acceleration_gen->log_client.get() != NULL){
				uniform_acceleration_gen->log_client->set_filename_prefix("conveyor-uniform-accel");
				uniform_acceleration_gen->log_client->set_connect();
			}
			uniform_acceleration_gen->Move();
		} else {
			log("ecp_t_conveyor_uniform_acceleration::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
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
	return new mrrocpp::ecp::conveyor::task::ecp_t_conveyor_uniform_acceleration(config);
}

} //namespace task

} //namespace common

} //namespace ecp

} //namespace mrrocpp
