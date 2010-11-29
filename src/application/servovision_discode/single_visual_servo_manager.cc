/*
 * single_visual_servo_manager.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "single_visual_servo_manager.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

single_visual_servo_manager::single_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> vs) :
	visual_servo_manager(ecp_task, section_name)
{
	servos.push_back(vs);
}

single_visual_servo_manager::~single_visual_servo_manager()
{
}

lib::Homog_matrix single_visual_servo_manager::get_aggregated_position_change()
{
	return servos[0]->get_position_change(get_current_position(), dt);
}

void single_visual_servo_manager::configure_all_servos()
{
	//logger::logDbg("single_visual_servo_manager::configure_all_servos()\n");
}

}//namespace generator

}

}

}
