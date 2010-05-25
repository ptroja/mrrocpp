/*
 * simple_visual_servo_manager.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "simple_visual_servo_manager.h"

#include "logger.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

simple_visual_servo_manager::simple_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
		visual_servo> vs) :
	visual_servo_manager(ecp_task, section_name)
{
	servos.push_back(vs);
}

simple_visual_servo_manager::~simple_visual_servo_manager()
{
}

lib::Homog_matrix simple_visual_servo_manager::get_aggregated_position_change()
{
	return servos[0]->get_position_change(get_current_position(), dt);
}

void simple_visual_servo_manager::configure_all_servos()
{
	//logDbg("simple_visual_servo_manager::configure_all_servos()\n");
}

}//namespace generator

}

}

}
