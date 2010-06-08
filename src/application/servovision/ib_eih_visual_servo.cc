/*
 * ib_eih_visual_servo.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "ib_eih_visual_servo.h"

#include <stdexcept>

using ecp_mp::sensor::fradia_sensor;
using namespace logger;
using namespace visual_servo_types;
using namespace std;
using namespace boost;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ib_eih_visual_servo::ib_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string & section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator)
{
	Eigen::Matrix <double, 1, 3> desired_translation;
	Eigen::Matrix <double, 3, 3> intrinsics;
	Eigen::Matrix <double, 1, 5> distortion;

	image_based_configuration ib_config;
	desired_translation = configurator.value <1, 3> ("desired_position_translation", section_name);
	ib_config.desired_position.x = desired_translation(0, 0);
	ib_config.desired_position.y = desired_translation(0, 1);
	ib_config.desired_position.z = desired_translation(0, 2);

	ib_config.desired_position.gamma = configurator.value <double> ("desired_position_rotation", section_name);

	intrinsics = configurator.value <3, 3> ("fradia_camera_intrinsics", section_name);

	distortion = configurator.value <1, 5> ("fradia_camera_distortion", section_name);

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			ib_config.dcp.intrinsics[i][j] = intrinsics(i, j);
		}
	}
	for (int i = 0; i < 5; ++i) {
		ib_config.dcp.distortion[i] = distortion(0, i);
	}

	ib_config.z_estimation_A = configurator.value <double> ("fradia_z_estimation_a", section_name);
	ib_config.z_estimation_B = configurator.value <double> ("fradia_z_estimation_b", section_name);
	ib_config.z_estimation_C = configurator.value <double> ("fradia_z_estimation_c", section_name);

	Eigen::Matrix <double, 3, 3> e_T_c;
	e_T_c = configurator.value <3, 3> ("e_t_c_rotation", section_name);

	double rot[3][3];

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rot[i][j] = e_T_c(i, j);
		}
	}

	e_T_c_position.set_rotation_matrix(rot);

	vsp_fradia = shared_ptr <ib_fradia_sensor> (new ib_fradia_sensor(configurator, section_name, ib_config));
}

ib_eih_visual_servo::~ib_eih_visual_servo()
{
}

lib::Homog_matrix ib_eih_visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix delta_position;

	lib::K_vector u_translation(0, 0, 0);
	lib::Homog_matrix u_rotation;
	Eigen::Matrix <double, 6, 1> e;
	Eigen::Matrix <double, 3, 1> e_translation;

	//	logDbg("ecp_g_ib_eih::next_step() 2\n");
	object_visible = vsp_fradia->get_reading_message().tracking;
	if (vsp_fradia->get_reading_message().tracking) {
		//		logDbg("ecp_g_ib_eih::next_step() 3\n");

		e(0, 0) = vsp_fradia->get_reading_message().error.x;
		e(1, 0) = vsp_fradia->get_reading_message().error.y;
		e(2, 0) = vsp_fradia->get_reading_message().error.z;
		e(3, 0) = vsp_fradia->get_reading_message().error.gamma;

		e_translation(0, 0) = e(0, 0);
		e_translation(1, 0) = e(1, 0);
		e_translation(2, 0) = e(2, 0);

		//		logDbg("ib_eih_visual_servo::get_position_change() %d, %d, %d, %+7.3lg\n", vsp_fradia->received_object.error.x, vsp_fradia->received_object.error.y, vsp_fradia->received_object.error.z, vsp_fradia->received_object.error.gamma);

		Eigen::Matrix <double, 6, 1> control;

		control = regulator->calculate_control(e, dt);
		//logDbg("ib_eih_visual_servo::get_position_change() control: %+07.3lg, %+07.3lg, %+07.3lg\n", control(0, 0), control(1, 0), control(2, 0));

		Eigen::Matrix <double, 3, 1> camera_to_object_translation;
		camera_to_object_translation(0, 0) = control(0, 0);
		camera_to_object_translation(1, 0) = control(1, 0);
		camera_to_object_translation(2, 0) = control(2, 0);

		u_translation = e_T_c_position * camera_to_object_translation;

		//		logDbg("ib_eih_visual_servo::get_position_change() u_translation: %+07.3lg, %+07.3lg, %+07.3lg\n", u_translation(0, 0), u_translation(1, 0), u_translation(2, 0));

		u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));
	}

	delta_position.set_rotation_matrix(u_rotation);
	delta_position.set_translation_vector(u_translation);

	//	logDbg("ib_eih_visual_servo::get_position_change() delta_position: %+07.3lg, %+07.3lg, %+07.3lg\n", delta_position(0, 3), delta_position(1, 3), delta_position(2, 3));

	return delta_position;
}

boost::shared_ptr <ecp_mp::sensor::sensor_interface> ib_eih_visual_servo::get_vsp_fradia()
{
	return boost::dynamic_pointer_cast <ecp_mp::sensor::sensor_interface>(vsp_fradia);
}

}//namespace generator

}

}

}
