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

pb_visual_servo::pb_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator)
{
	lib::Homog_matrix E_T_G_desired = configurator.value <3, 4> ("E_T_G_desired", section_name);
	G_T_E_desired = !E_T_G_desired;

	position_based_configuration pb_config;

	Eigen::Matrix <double, 3, 3> intrinsics = configurator.value <3, 3> ("fradia_camera_intrinsics", section_name);

	Eigen::Matrix <double, 1, 5> distortion = configurator.value <1, 5> ("fradia_camera_distortion", section_name);

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			pb_config.dcp.intrinsics[i][j] = intrinsics(i, j);
		}
	}
	for (int i = 0; i < 5; ++i) {
		pb_config.dcp.distortion[i] = distortion(0, i);
	}

	vsp_fradia = boost::shared_ptr <pb_fradia_sensor>(new pb_fradia_sensor(configurator, section_name, pb_config));
}

pb_visual_servo::~pb_visual_servo()
{
}

boost::shared_ptr <ecp_mp::sensor::sensor_interface> pb_visual_servo::get_vsp_fradia()
{
	return boost::dynamic_pointer_cast <ecp_mp::sensor::sensor_interface>(vsp_fradia);
}

bool pb_visual_servo::is_object_visible_in_latest_reading()
{
	return vsp_fradia->get_reading_message().tracking;
}

lib::sensor::VSP_REPORT_t pb_visual_servo::get_sensor_report()
{
	return vsp_fradia->get_report();
}


}//namespace generator {

}

}
