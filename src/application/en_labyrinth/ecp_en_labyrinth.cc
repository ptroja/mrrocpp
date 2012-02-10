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

	sr_ecp_msg->message("ECP en_labyrinth");
};


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


