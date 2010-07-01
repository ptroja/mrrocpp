/*
 * ecp_t_objectfollower_pb_sac.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_objectfollower_pb_sac.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac(
		mrrocpp::lib::configurator& config) :
	task(config) {
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);

	char config_section_name[] = { "[object_follower_sac_1]" };

	log_dbg_enabled = true;

	Eigen::Matrix <double, 3, 1> p1, p2;
	p1(0, 0) = 0.7;
	p1(1, 0) = -0.3;
	p1(2, 0) = 0.0;

	p2(0, 0) = 0.95;
	p2(1, 0) = 0.3;
	p2(2, 0) = 0.7;

	shared_ptr<position_constraint> cube(new cubic_constraint(p1, p2));
	reg = shared_ptr<visual_servo_regulator> (new regulator_p(config,
			config_section_name));
	vs = shared_ptr<visual_servo> (new pb_sac_visual_servo(reg,
			config_section_name, config));
	sm = shared_ptr<simple_visual_servo_manager> (
			new simple_visual_servo_manager(*this, config_section_name, vs));
	sm->add_position_constraint(cube);
	sm->configure();
}

ecp_t_objectfollower_pb_sac::~ecp_t_objectfollower_pb_sac() {
	delete ecp_m_robot;
}

void ecp_t_objectfollower_pb_sac::main_task_algorithm() {
	sm->Move();
	ecp_termination_notice();
}

} // namespace task

} // namespace irp6ot

namespace common {
namespace task {
task* return_created_ecp_task(lib::configurator &config) {
	return new irp6ot_m::task::ecp_t_objectfollower_pb_sac(config);
}
} // namespace task
} // namespace common

} // namespace ecp

} // namespace mrrocpp
