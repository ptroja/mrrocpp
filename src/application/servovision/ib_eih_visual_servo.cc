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

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ib_eih_visual_servo::ib_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const char* section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator)
{
	try {
		vsp_fradia
				= boost::shared_ptr <fradia_sensor <image_based_reading, image_based_configuration> >(new fradia_sensor <
						image_based_reading, image_based_configuration> (configurator, section_name));

		Eigen::Matrix <double, 1, 3> desired_translation;
		Eigen::Matrix <double, 3, 3> intrinsics;
		Eigen::Matrix <double, 1, 5> distortion;

		image_based_configuration ib_config;
		ib_config.set_parameters = true;

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

		vsp_fradia->configure_fradia_task(ib_config);

		log("ib_eih_visual_servo::ib_eih_visual_servo(): after vsp_fradia->configure_fradia_task\n");

		Eigen::Matrix <double, 3, 3> e_T_c;
		e_T_c = configurator.value <3, 3> ("e_t_c_rotation", section_name);

		double rot[3][3];

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				rot[i][j] = e_T_c(i, j);
			}
		}

		e_T_c_position.set_rotation_matrix(rot);
	} catch (const exception& e) {
		printf("ib_eih_visual_servo::ib_eih_visual_servo(): %s\n", e.what());
		throw e;
	}
}

ib_eih_visual_servo::~ib_eih_visual_servo()
{
}

lib::Homog_matrix ib_eih_visual_servo::get_position_change(const lib::Homog_matrix& current_position)
{
	//	logDbg("ecp_g_ib_eih::next_step() begin\n");

	lib::Homog_matrix delta_position;

	lib::K_vector u_translation(0, 0, 0);
	lib::Homog_matrix u_rotation;
	Eigen::Matrix <double, 6, 1> e;
	Eigen::Matrix <double, 3, 1> e_translation;

	//	logDbg("ecp_g_ib_eih::next_step() 2\n");
	object_visible = vsp_fradia->received_object.tracking;
	if (vsp_fradia->received_object.tracking) {
		//		logDbg("ecp_g_ib_eih::next_step() 3\n");

		e(0, 0) = vsp_fradia->received_object.error.x;
		e(1, 0) = vsp_fradia->received_object.error.y;
		e(2, 0) = vsp_fradia->received_object.error.z;
		//e(2, 0) = 0;
		e(3, 0) = vsp_fradia->received_object.error.gamma;

		e_translation(0, 0) = e(0, 0);
		e_translation(1, 0) = e(1, 0);
		e_translation(2, 0) = e(2, 0);

		//		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.error.x: %g\n", (double) vsp_fradia->received_object.error.x);
		//		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.error.y: %g\n", (double) vsp_fradia->received_object.error.y);
		//		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.error.z: %g\n", (double) vsp_fradia->received_object.error.z);
		//		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.error.alpha: %g\n", vsp_fradia->received_object.error.gamma);

		//		logDbg("ecp_g_ib_eih::next_step() 4\n");

		Eigen::Matrix <double, 6, 1> control;

		//		logDbg("ecp_g_ib_eih::next_step() 4a\n");

		control = regulator->calculate_control(e);

		//		logDbg("ecp_g_ib_eih::next_step() 5\n");

		Eigen::Matrix <double, 3, 1> camera_to_object_translation;
		camera_to_object_translation(0, 0) = control(0, 0);
		camera_to_object_translation(1, 0) = control(1, 0);
		camera_to_object_translation(2, 0) = control(2, 0);

		//		logDbg("ecp_g_ib_eih::next_step() 6\n");

		u_translation = e_T_c_position * camera_to_object_translation;

		//		logDbg("ecp_g_ib_eih::next_step() 7\n");

		//		logDbg("e[3] = %g\t", e(3, 0));
		//		logDbg("control [3] = %g\n", control(3, 0));

		u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));

		//		logDbg("ecp_g_ib_eih::next_step() 9\n");
		//		for (int i = 0; i < 4; ++i) {
		//			logDbg("e[%d] = %g\t", i, e(i, 0));
		//		}
		//		logDbg("\n");
		//		for (int i = 0; i < 3; ++i) {
		//			logDbg("u_translation[%d] = %g\t", i, u_translation(i, 0));
		//		}
		//		logDbg("\n");

		//logDbg("Tracking.\n");
	}

	//	logDbg("ecp_g_ib_eih::next_step() 10\n");

	delta_position.set_rotation_matrix(u_rotation);
	delta_position.set_translation_vector(u_translation);

	return delta_position;
}

boost::shared_ptr <mrrocpp::lib::sensor> ib_eih_visual_servo::get_vsp_fradia()
{
	return boost::dynamic_pointer_cast <mrrocpp::lib::sensor>(vsp_fradia);
}

}//namespace generator

}

}

}
