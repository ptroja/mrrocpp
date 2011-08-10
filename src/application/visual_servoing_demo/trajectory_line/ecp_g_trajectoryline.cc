/*
 * ecp_g_trajectoryline.cc
 *
 *  Created on: 11-08-2011
 *      Author: mateusz
 */

#include "ecp_g_trajectoryline.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

ecp_g_trajectory_line::ecp_g_trajectory_line(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
		common::generator::generator(ecp_task)
{
	// TODO Auto-generated constructor stub

}

ecp_g_trajectory_line::~ecp_g_trajectory_line()
{
	// TODO Auto-generated destructor stub
}

bool ecp_g_trajectory_line::first_step()
{
	return false;
}

bool ecp_g_trajectory_line::next_step()
{
	return false;
}

} /* namespace generator */
} /* namespace common */
} /* namespace ecp */
} /* namespace mrrocpp */
