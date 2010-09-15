/*
 * ecp_t_vs_tutorial.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "ecp_t_vs_tutorial.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_t_vs_tutorial::ecp_t_vs_tutorial(mrrocpp::lib::configurator& configurator) :
	task(configurator)
{
	vs_config_section_name = "[object_follower_ib]";

	std::string robot_name = configurator.value <std::string> ("robot_name", vs_config_section_name);
	if (robot_name == "irp6ot") {
		ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	} else if (robot_name == "irp6p") {
		ecp_m_robot = new ecp::irp6p_m::robot(*this);
	} else {
		throw std::logic_error("Unknown robot: " + robot_name);
	}

	log_dbg_enabled = true;
}

ecp_t_vs_tutorial::~ecp_t_vs_tutorial()
{
	delete ecp_m_robot;
}

void ecp_t_vs_tutorial::main_task_algorithm(void)
{
	move_visual_servo();

	ecp_termination_notice();
}

void ecp_t_vs_tutorial::move_visual_servo()
{
	sr_ecp_msg->message("Creating visual servo...");

	shared_ptr <position_constraint> cube(new cubic_constraint(config, vs_config_section_name));

	reg = shared_ptr <visual_servo_regulator> (new regulator_p(config, vs_config_section_name));

	vs = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg, vs_config_section_name, config));

	object_reached_term_cond
			= shared_ptr <termination_condition> (new object_reached_termination_condition(config, vs_config_section_name));

	timeout_term_cond = shared_ptr <termination_condition> (new timeout_termination_condition(5));

	sm = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, vs_config_section_name.c_str(), vs));

	sm->add_position_constraint(cube);

	//sm->add_termination_condition(object_reached_term_cond);
	//sm->add_termination_condition(timeout_term_cond);

	sm->configure();

	sm->Move();

	if (object_reached_term_cond->is_condition_met()) {
		sr_ecp_msg->message("object_reached_term_cond is met");
	} else {
		sr_ecp_msg->message("object_reached_term_cond IS NOT MET");
	}

	if (timeout_term_cond->is_condition_met()) {
		sr_ecp_msg->message("timeout_term_cond is met");
	} else {
		sr_ecp_msg->message("timeout_term_cond IS NOT MET");
	}
}

task* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_vs_tutorial(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
