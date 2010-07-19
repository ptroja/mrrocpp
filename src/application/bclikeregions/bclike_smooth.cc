/*
 * bclike_smooth.cpp
 *
 *  Created on: 05-07-2010
 *      Author: kszkudla
 */

#include "bclike_smooth.h"
#include "bclikeregions_task.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task, bool synchronized) :
		common::generator::smooth(ecp_task, synchronized){
	// TODO Auto-generated constructor stub
	vsp_fradia = boost::shared_ptr<task::bcl_fradia_sensor>();//bcl.get_vsp_fradia();

}

bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bclikeregions_task & task, bool synchronized):
				common::generator::smooth((mrrocpp::ecp::common::task::task &)task, synchronized){
	vsp_fradia = boost::shared_ptr<task::bcl_fradia_sensor>(task.get_vsp_fradia());

}


bclike_smooth::~bclike_smooth() {
	// TODO Auto-generated destructor stub
}

//set necessary instructions, and other data for preparing the robot
bool bclike_smooth::first_step(){

	std::cout << "FIRST STEP" << std::endl;

	return smooth::first_step();
}

bool bclike_smooth::next_step(){

	return smooth::next_step();

}

}

}

}

}
