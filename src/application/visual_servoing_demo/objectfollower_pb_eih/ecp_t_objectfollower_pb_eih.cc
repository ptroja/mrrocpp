/*
 * ecp_t_objectfollower_pb_eih.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include <stdexcept>

#include "ecp_t_objectfollower_pb_eih.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

#include "../ecp_mp_g_visual_servo_tester.h"

using namespace std;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_t_objectfollower_pb_eih::ecp_t_objectfollower_pb_eih(mrrocpp::lib::configurator& config) :
	common::task::task(config)
{
	try{
		std::string robot_name = config.value<std::string>("robot_name", "[visualservo_tester]");
		if(robot_name == lib::irp6p_m::ROBOT_NAME){
			ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
		} else if(robot_name == lib::irp6ot_m::ROBOT_NAME){
			ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
		} else {
			throw std::runtime_error("ecp_t_objectfollower_pb_eih option robot_name in config file has unknown value: " + robot_name);
		}

		char config_section_name[] = { "[object_follower_pb]" };

		log_enabled = true;
		log_dbg_enabled = true;

		boost::shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));

		if(config.exists_and_true("use_pid_regulator", config_section_name)){
			sr_ecp_msg->message("Using PID regulator");
			reg = boost::shared_ptr <visual_servo_regulator> (new regulator_pid(config, config_section_name));
		} else {
			sr_ecp_msg->message("Using P regulator");
			reg = boost::shared_ptr <visual_servo_regulator> (new regulator_p(config, config_section_name));
		}

		sr_ecp_msg->message("Creating DisCODe sensor");
		boost::shared_ptr <discode_sensor> ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));
		vs = boost::shared_ptr <visual_servo> (new pb_eih_visual_servo(reg, ds, config_section_name, config));

		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 3\n");
		sm = boost::shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));
		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 4\n");
		sm->add_position_constraint(cube);
		log_dbg("ecp_t_objectfollower_pb: configuring visual_servo_manager\n");
		sm->configure();
	}catch(std::exception& ex){
		sr_ecp_msg->message(lib::FATAL_ERROR, string("ERROR in ecp_t_objectfollower_pb_eih: ") + ex.what());
		throw ex;
	}
	log_dbg("ecp_t_objectfollower_pb: initialization completed.\n");
}

void ecp_t_objectfollower_pb_eih::main_task_algorithm(void)
{
	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {
			if(sm->log_client.get() != NULL){
				sm->log_client->set_filename_prefix("pb-eih_vs_manager");
				sm->log_client->set_connect();
			}
			if(vs->log_client.get() != NULL){
				vs->log_client->set_filename_prefix("pb-eih_vs");
				vs->log_client->set_connect();
			}
			sm->Move();
		} else {
			log("ecp_t_objectfollower_pb::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
	}

	termination_notice();
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_objectfollower_pb_eih(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
