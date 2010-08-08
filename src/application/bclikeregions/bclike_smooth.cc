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
		common::generator::newsmooth(ecp_task, lib::ECP_JOINT, 7),
		bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	vsp_fradia = NULL;
	no_fradia = true;
}

bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task):
				common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, lib::ECP_JOINT, 7),
				bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	std::cout << "FRADIA VERSION" << std::endl;
	no_fradia = false;
	vsp_fradia = bcl_ecp.get_vsp_fradia();


	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD no vsp constructor" << std::endl;
	}

}

bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task, task::bcl_fradia_sensor* fr):
						common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, lib::ECP_JOINT, 7),
						bcl_ecp((task::bcl_t_switcher &)ecp_t),
						vsp_fradia(fr), num_send(0){

	no_fradia = false;

	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;//.get();
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD vsp constructor" << std::endl;
	}
}

bclike_smooth::~bclike_smooth() {
	// TODO Auto-generated destructor stub
}

//set necessary instructions, and other data for preparing the robot
bool bclike_smooth::first_step(){

	std::cout << "FIRST STEP" << std::endl;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;

	if(no_fradia){
		std::cout << "ERROR: no fradia == TRUE" << std::endl;
		return false;
	}

	return newsmooth::first_step();
}

bool bclike_smooth::next_step(){

	reading = bcl_ecp.vsp_fradia->get_reading_message();

	std::vector<double> vec;
	vec.assign(the_robot->reply_package.arm.pf_def.arm_coordinates, the_robot->reply_package.arm.pf_def.arm_coordinates + 7);

	if(reading.code_found)
		translateToRobotPosition(reading);

#ifdef SINGLE_MOVE //robot's move to the end, and then found codes are sent to MP

	msg.addFradiaOrderToVector(reading, readings);

	if(newsmooth::next_step())
		return true;

	if(num_send < readings.size())
		strcpy(ecp_t.ecp_reply.ecp_2_mp_string, msg.regionsVectorToString(readings, num_send));
	else
		strcpy(ecp_t.ecp_reply.ecp_2_mp_string, "KONIEC");


	return false;

#else //robot's move stopped each time when code detected


	if(reading.code_found){
		strcpy(ecp_t.ecp_reply.ecp_2_mp_string, msg.fradiaOrderToString(reading, vec));
		return false;
	}

	if(newsmooth::next_step())
		return true;
	else{
		strcpy(ecp_t.ecp_reply.ecp_2_mp_string, "KONIEC");
		return false;
	}

#endif

}

void bclike_smooth::translateToRobotPosition(task::fradia_regions& regs){

}

}

}

}

}
