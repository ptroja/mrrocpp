/*
 * ecp_t_objectfollower_pb_sac.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_objectfollower_pb_sac.h"

#include "../ecp_mp_g_visual_servo_tester.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac(mrrocpp::lib::configurator& config) :
	task(config)
{
	ecp_m_robot = new ecp::irp6p_m::robot(*this);

	char config_section_name[] = { "[object_follower_sac_1]" };

	log_dbg_enabled = true;

	shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));
	reg = shared_ptr <visual_servo_regulator> (new regulator_p(config, config_section_name));
	vs = shared_ptr <visual_servo> (new pb_sac_visual_servo(reg, config_section_name, config));
	sm = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));
	sm->add_position_constraint(cube);
	sm->configure();
}

ecp_t_objectfollower_pb_sac::~ecp_t_objectfollower_pb_sac()
{
	delete ecp_m_robot;
}

void ecp_t_objectfollower_pb_sac::main_task_algorithm()
{
	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {
			sm->Move();
		} else {
			log("ecp_t_objectfollower_pb::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
	}

	ecp_termination_notice();
}

} // namespace task

} // namespace irp6ot

namespace common {
namespace task {
task* return_created_ecp_task(lib::configurator &config)
{
	return new irp6ot_m::task::ecp_t_objectfollower_pb_sac(config);
}
} // namespace task
} // namespace common

} // namespace ecp

} // namespace mrrocpp
