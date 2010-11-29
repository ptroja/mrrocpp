/*
 * pb_visual_servo.cc
 *
 *  Created on: May 26, 2010
 *      Author: mboryn
 */

#include "pb_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

pb_visual_servo::pb_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <
		mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator, sensor)
{
	lib::Homog_matrix E_T_G_desired = configurator.value <3, 4> ("E_T_G_desired", section_name);
	G_T_E_desired = !E_T_G_desired;
}

pb_visual_servo::~pb_visual_servo()
{
}

void pb_visual_servo::retrieve_reading()
{
	reading = sensor->get_received_object <Processors::VisualServoPB::PBReading> ();
}

bool pb_visual_servo::is_object_visible_in_latest_reading()
{
	return reading.objectVisible;
}

}//namespace generator {

}

}
