/*
 * bclike_smooth.cpp
 *
 *  Created on: 05-07-2010
 *      Author: kszkudla
 */

#include "bclike_smooth.h"
#include "bclikeregions_task.h"
#include "bcl_t_switcher.h"
#include <cstring>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task) :
		common::generator::newsmooth(ecp_task, lib::ECP_JOINT, 8),
		bcl_ecp((task::bcl_t_switcher &)ecp_t){
//		vsp_fradia(vsp_fradia){

//	vsp_fradia = boost::shared_ptr<task::bcl_fradia_sensor>();//bcl.get_vsp_fradia();
	vsp_fradia = NULL;
	no_fradia = true;
}

bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bclikeregions_task & task):
				common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, lib::ECP_JOINT, 8),
				bcl_ecp((task::bcl_t_switcher &)ecp_t){
//				vsp_fradia(vsp_fradia){
	std::cout << "FRADIA VERSION" << std::endl;
	no_fradia = false;
//	vsp_fradia = boost::shared_ptr<task::bcl_fradia_sensor>(bcl_ecp.get_vsp_fradia());
	vsp_fradia = bcl_ecp.get_vsp_fradia();


	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_CVFRADIA] = vsp_fradia;//.get();
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD" << std::endl;
	}

}

//bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bclikeregions_task & task, shared_ptr<task::bcl_fradia_sensor> fr):
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bclikeregions_task & task, task::bcl_fradia_sensor* fr):
						common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, lib::ECP_JOINT, 8),
						bcl_ecp((task::bcl_t_switcher &)ecp_t),
						vsp_fradia(fr){

	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_CVFRADIA] = vsp_fradia;//.get();
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD" << std::endl;
	}
}

bclike_smooth::~bclike_smooth() {
	// TODO Auto-generated destructor stub
}

//set necessary instructions, and other data for preparing the robot
bool bclike_smooth::first_step(){

	std::cout << "FIRST STEP" << std::endl;

	if(no_fradia){
		std::cout << "ERROR: no fradia == TRUE" << std::endl;
		return false;
	}

	return newsmooth::first_step();
}

bool bclike_smooth::next_step(){

//	reading = bcl_ecp.vsp_fradia->get_reading_message();

//	bcl_ecp.robot_m[lib::ROBOT_IRP6OT_M]->ecp_replay_package.ecp_2_mp_string

	return newsmooth::next_step();

}

}

}

}

}
