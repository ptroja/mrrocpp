/*
 * ecp_en_labyrinth.cc
 *
 * Author: enatil
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp_mp_g_en_labyrinth.h"
#include "ecp_g_en_labyrinth.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "base/ecp/ecp_task.h"
//#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
//#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
//#include "generator/ecp/ecp_mp_g_tfg.h"

#include "ecp_en_labyrinth.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_en_labyrinth::ecp_en_labyrinth(lib::configurator &_config): common::task::task(_config)
{

	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);

	register_generator(new common::generator::en_labyrinth(*this));

	gp = new common::generator::get_position(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);
	register_generator(gp);

	register_generator(new common::generator::bias_edp_force(*this));

	{
		common::generator::tff_nose_run *ecp_gen = new common::generator::tff_nose_run(*this, 8);
		ecp_gen->configure_pulse_check(true);
		ecp_gen->configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION);
		register_generator(ecp_gen);
	}


	sr_ecp_msg->message("ECP en_labyrinth");
};
/*
void ecp_en_labyrinth::mp_2_ecp_next_state_string_handler(void)
{
	gp->Move();

	std::vector <double> position_vector;
	position_vector = gp->get_position_vector();

	if(!position_vector.size()) {
		sr_ecp_msg->message("get_position_vector is empty!");
	}

	std::cout << "get_position generator:" << std::endl;
	for(size_t i = 0; i < position_vector.size(); ++i) {
		std::cout << position_vector[i] << std::endl;
	}
	std::cout << std::endl;

}
*/
}
}

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new ecp_en_labyrinth(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


