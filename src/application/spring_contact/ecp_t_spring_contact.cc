/*!
 * @file
 * @brief File contains ecp_task class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include <cstdio>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/irp6p_m/const_irp6p_m.h"

#include "ecp_t_spring_contact.h"

#include "ecp_st_spring_contact.h"
#include "subtask/ecp_st_bias_edp_force.h"
#include "subtask/ecp_st_tff_nose_run.h"

#include "subtask/ecp_mp_st_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
spring_contact::spring_contact(lib::configurator &_config) :
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

	// utworzenie podzadan
	{
		sub_task::sub_task* ecpst;
		ecpst = new sub_task::spring_contact(*this);
		subtask_m[ecp_mp::sub_task::SPRING_CONTACT] = ecpst;

		ecpst = new sub_task::bias_edp_force(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = ecpst;
	}

	{
		sub_task::tff_nose_run* ecpst;
		ecpst = new sub_task::tff_nose_run(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_TFF_NOSE_RUN] = ecpst;
		ecpst->nrg->configure_pulse_check(true);
		ecpst->nrg->configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION);
	}

	sr_ecp_msg->message("ecp spring_contact loaded");
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::spring_contact(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
