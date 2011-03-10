/*
 * t_mm_test.cc
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>


#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "t_mm_test.h"
//#include "base/lib/datastr.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
mm_test::mm_test(lib::configurator &_config): common::task::task(_config)
{
	if (config.section_name == lib::irp6ot_m::ECP_SECTION) {
			ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6ot_m::robot(*this);
			sg = new common::generator::newsmooth(*this,lib::ECP_JOINT, 7);
		} else if (config.section_name == lib::irp6p_m::ECP_SECTION) {
			ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6p_m::robot(*this);
			sg = new common::generator::newsmooth(*this,lib::ECP_JOINT, 6);
		} else {
			// TODO: throw, robot unsupported
			return;
		}

	//my_generator = new generator::g_mm_test(*this);
	sr_ecp_msg->message("ECP loaded mm_test");
};

void mm_test::main_task_algorithm(void ) {

	sr_ecp_msg->message("max's test ready");

	get_next_state();
	std::string path(mrrocpp_network_path);

	path += (char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string;

	sg->load_trajectory_from_file(path.c_str());
	sg->calculate_interpolate();
	sg->Move();
	sr_ecp_msg->message("moved");

	ecp_termination_notice();
	sr_ecp_msg->message("noticed");
};

}
} // namespace irp6ot

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new mm_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


