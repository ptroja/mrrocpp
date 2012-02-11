/*!
 * @file
 * @brief File contains ecp_task class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include "ecp_t_spring_contact.h"

// generators to be register headers
#include "ecp_g_spring_contact.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_g_tff_nose_run.h"

// ecp_robots headers
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

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
		throw std::runtime_error("Robot not supported");
	}

	// utworzenie generatorow do uruchamiania dispatcherem
	register_generator(new common::generator::bias_edp_force(*this));

	{
		common::generator::tff_nose_run *ecp_gen = new common::generator::tff_nose_run(*this, 8);
		ecp_gen->configure_pulse_check(true);
		ecp_gen->configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION);
		register_generator(ecp_gen);
	}

	register_generator(new generator::spring_contact(*this, 5));

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
