/*!
 * @file
 * @brief File contains edge_follow_mr ecp_task class definition of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include <cstdio>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/irp6p_m/const_irp6p_m.h"

#include "ecp_t_edge_follow_mr.h"

#include "generator/ecp/force/ecp_g_bias_edp_force.h"
#include "generator/ecp/force/ecp_g_tff_nose_run.h"
#include "ecp_g_edge_follow.h"

#include "generator/ecp/force/ecp_mp_g_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
edge_follow_mr::edge_follow_mr(lib::configurator &_config) :
		common::task::task(_config)
{
	// the robot is choose depending on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		// TODO: throw
	}

	// utworzenie generatorow do uruchamiania dispatcherem
	generator_m[ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE] = new generator::bias_edp_force(*this);

	{
		generator::tff_nose_run *ecp_gen = new generator::tff_nose_run(*this, 8);
		ecp_gen->configure_pulse_check(true);
		generator_m[ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN] = ecp_gen;
	}

	generator_m[ecp_mp::generator::EDGE_FOLLOW] = new generator::y_edge_follow_force(*this, 8);

	// utworzenie podzadan

	sr_ecp_msg->message("ecp edge_follow_MR loaded");
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::edge_follow_mr(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
