/*
 * ecp_t_objectfollower_ib_eih.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "ecp_t_objectfollower_ib_eih.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

#include "../ecp_mp_g_visual_servo_tester.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(mrrocpp::lib::configurator& configurator) :
		common::task::task(configurator)
{
	std::string robot_name = config.value<std::string>("robot_name", "[visualservo_tester]");
	if(robot_name == lib::irp6p_m::ROBOT_NAME){
		ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
	} else if(robot_name == lib::irp6ot_m::ROBOT_NAME){
		ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
	} else {
		throw std::runtime_error("ecp_t_objectfollower_ib_eih option robot_name in config file has unknown value: " + robot_name);
	}

	char config_section_name[] = { "[object_follower_ib]" };

	log_dbg_enabled = true;
	log_enabled = true;

	shared_ptr <position_constraint> cube(new cubic_constraint(configurator, config_section_name));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 1\n");
	reg = shared_ptr <visual_servo_regulator> (new regulator_p(configurator, config_section_name));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 2\n");

	boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> ds = boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(configurator, config_section_name));
	vs = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg, ds, config_section_name, configurator));

	term_cond = shared_ptr <termination_condition> (new object_reached_termination_condition(configurator, config_section_name));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 3\n");
	sm = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 4\n");
	sm->add_position_constraint(cube);

	//sm->add_termination_condition(term_cond);
	//log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 5\n");

	sm->configure();
	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 6\n");
}

void ecp_t_objectfollower_ib_eih::main_task_algorithm(void)
{
	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {
			sm->Move();
		} else {
			log("ecp_t_objectfollower_ib_eih::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
	}

	termination_notice();
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_objectfollower_ib_eih(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
