/*
 * ecp_t_objectfollower_pb_sac.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_t_objectfollower_pb_sac.h"

#include "../ecp_mp_g_visual_servo_tester.h"

using boost::shared_ptr;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac(mrrocpp::lib::configurator& config) :
	mrrocpp::ecp::common::task::task(config)
{
	log_dbg_enabled = true;
	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() begin\n");

	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);

	char config_section_name[] = { "[object_follower_sac_1]" };

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 1\n");

	boost::shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 2\n");

	reg = boost::shared_ptr <visual_servo_regulator> (new regulator_p(config, config_section_name));

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 3 \n");

	boost::shared_ptr <discode_sensor> ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 4\n");

	vs = boost::shared_ptr <visual_servo> (new pb_sac_visual_servo(reg, ds, config_section_name, config));

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 5\n");

	sm = boost::shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 6\n");

	sm->add_position_constraint(cube);

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() 7\n");

	sm->configure();

	log_dbg("ecp_t_objectfollower_pb_sac::ecp_t_objectfollower_pb_sac() end\n");
}

void ecp_t_objectfollower_pb_sac::main_task_algorithm()
{
	log_dbg("ecp_t_objectfollower_pb_sac::main_task_algorithm() begin\n");
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
task_base* return_created_ecp_task(lib::configurator &config)
{
	return new irp6ot_m::task::ecp_t_objectfollower_pb_sac(config);
}
} // namespace task
} // namespace common

} // namespace ecp

} // namespace mrrocpp
