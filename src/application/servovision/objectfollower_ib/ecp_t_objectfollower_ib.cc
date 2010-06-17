/*
 * ecp_t_objectfollower_ib.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "ecp_t_objectfollower_ib.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

const double
		ecp_t_objectfollower_ib::initial_position_joints[MAX_SERVOS_NR] = { 0.0, 0.0, -1.428, 0.0, 0.0, 4.720, 0.0, 0.075 };

ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(mrrocpp::lib::configurator& configurator) :
	task(configurator)
{
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	//ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	smooth_gen = shared_ptr <smooth> (new smooth(*this, true));

	char config_section_name[] = { "[object_follower_1]" };

	log_dbg_enabled = true;

	Eigen::Matrix <double, 3, 1> p1, p2;
	p1(0, 0) = 0.6;
	p1(1, 0) = -0.4;
	p1(2, 0) = 0.1;

	p2(0, 0) = 0.95;
	p2(1, 0) = 0.4;
	p2(2, 0) = 0.3;

	shared_ptr <position_constraint> cube(new cubic_constraint(p1, p2));

	log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 1\n");
	reg = shared_ptr <visual_servo_regulator> (new regulator_p(configurator, config_section_name));

	log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 2\n");
	vs = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg, config_section_name, configurator));

	term_cond = shared_ptr<termination_condition>(new object_reached_termination_condition(0.005, 0.005, 50));

	log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 3\n");
	sm = shared_ptr <simple_visual_servo_manager> (new simple_visual_servo_manager(*this, config_section_name, vs));

	log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 4\n");
	sm->add_position_constraint(cube);

	//sm->add_termination_condition(term_cond);
	//log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 5\n");

	sm->configure();
	log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 6\n");
}

ecp_t_objectfollower_ib::~ecp_t_objectfollower_ib()
{
	delete ecp_m_robot;
}

void ecp_t_objectfollower_ib::main_task_algorithm(void)
{
	log_dbg("ecp_t_objectfollower_ib::main_task_algorithm(void) begin\n");
	//moveToInitialPosition();
	log("ecp_t_objectfollower_ib::main_task_algorithm(void) 1\n");
	sm->Move();
	log("ecp_t_objectfollower_ib::main_task_algorithm(void) 2\n");

	ecp_termination_notice();
	log_dbg("ecp_t_objectfollower_ib::main_task_algorithm(void) end\n");
}

void ecp_t_objectfollower_ib::moveToInitialPosition()
{
	double a[MAX_SERVOS_NR] = { 0.15, 0.15, 0.5, 0.15, 0.15, 0.15, 0.15, 0.001 };
	double v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.20, 0.01 };


	smooth_gen->reset();
	smooth_gen->set_absolute();
	smooth_gen->load_coordinates(lib::ECP_JOINT, v, a, (double *) initial_position_joints, true);
	smooth_gen->Move();
}

task* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_objectfollower_ib(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
