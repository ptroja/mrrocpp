/*
 * pb_eih_visual_servo.cc
 *
 *  Created on: May 11, 2010
 *      Author: mboryn
 */

#include "pb_eih_visual_servo.h"

#include <iostream>

using ecp_mp::sensor::fradia_sensor;
using namespace logger;
using namespace visual_servo_types;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

pb_eih_visual_servo::pb_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	pb_visual_servo(regulator), max_steps_without_reading(10), steps_without_reading(0)
{
	try {
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
		//vsp_fradia->configure_sensor();

		E_T_C = configurator.value <3, 4> ("E_T_C", section_name);

		lib::Homog_matrix E_T_G_desired = configurator.value <3, 4> ("E_T_G_desired", section_name);
		G_T_E_desired = !E_T_G_desired;
	} catch (const exception& e) {
		printf("pb_eih_visual_servo::pb_eih_visual_servo(): %s\n", e.what());
		throw e;
	}
}

pb_eih_visual_servo::~pb_eih_visual_servo()
{
}

lib::Homog_matrix pb_eih_visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix delta_position;

	//	log_dbg("pb_eih_visual_servo::get_position_change(): report: %d\n", vsp_fradia->get_report());

	if (vsp_fradia->get_report() == lib::VSP_SENSOR_NOT_CONFIGURED) {	// sensor not yet ready
		return delta_position;
	} else if (vsp_fradia->get_report() == lib::VSP_READING_NOT_READY) { // maybe there was a reading
		if (steps_without_reading > max_steps_without_reading) { // but if it was too long ago
			object_visible = false;	// we have to consider object not longer visible
			log_dbg("pb_eih_visual_servo::get_position_change(): object considered no longer visible\n");
			return delta_position;
		} else {
			steps_without_reading++;
		}
	} else if (vsp_fradia->get_report() == lib::VSP_REPLY_OK) { // we have a reading
		steps_without_reading = 0; // reset counter
	}

	object_visible = vsp_fradia->get_reading_message().tracking;

	if (object_visible) {
		lib::Homog_matrix C_T_G(vsp_fradia->get_reading_message().position);
		//		cout << "C_T_G:\n" << C_T_G;
		//		cout.flush();

		lib::Homog_matrix error_matrix;

		error_matrix = G_T_E_desired * E_T_C * C_T_G;

		lib::Xyz_Angle_Axis_vector aa_vector;
		error_matrix.get_xyz_angle_axis(aa_vector);

		//		logDbg("aa_vector: %10lg, %10lg, %10lg, %10lg, %10lg, %10lg\n", aa_vector(0, 0), aa_vector(1, 0), aa_vector(2, 0), aa_vector(3, 0), aa_vector(4, 0), aa_vector(5, 0));

		aa_vector = regulator->calculate_control(aa_vector, dt);
		//		logDbg("aa_vector after regulation: %10lg, %10lg, %10lg, %10lg, %10lg, %10lg\n", aa_vector(0, 0), aa_vector(1, 0), aa_vector(2, 0), aa_vector(3, 0), aa_vector(4, 0), aa_vector(5, 0));

		delta_position.set_from_xyz_angle_axis(aa_vector);
	}

	return delta_position;
}

boost::shared_ptr <ecp_mp::sensor::sensor_interface> pb_eih_visual_servo::get_vsp_fradia()
{
	return boost::dynamic_pointer_cast <ecp_mp::sensor::sensor_interface>(vsp_fradia);
}

} // namespace generator

}

}

}
