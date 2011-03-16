/*
 * ib_eih_visual_servo.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "ib_eih_visual_servo.h"

#include <stdexcept>

using namespace logger;
using namespace std;
using namespace mrrocpp::ecp_mp::sensor::discode;

namespace mrrocpp {

namespace ecp {

namespace servovision {

ib_eih_visual_servo::ib_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <
		mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string & section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator, sensor)
{
	desired_position = configurator.value <4, 1> ("desired_position", section_name);

	Eigen::Matrix <double, 3, 4> e_T_c;
	e_T_c.setZero();
	e_T_c.block(0,0,3,3) = configurator.value <3, 3> ("e_t_c_rotation", section_name);
	e_T_c_position = lib::Homog_matrix(e_T_c);

	reading.objectVisible = false;
}

ib_eih_visual_servo::~ib_eih_visual_servo()
{
}

lib::Homog_matrix ib_eih_visual_servo::compute_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::K_vector u_translation(0, 0, 0);
	lib::Homog_matrix u_rotation;
	Eigen::Matrix <double, 6, 1> e;
	Eigen::Matrix <double, 3, 1> e_translation;

	e.setZero();

	Eigen::Matrix <double, Types::ImagePosition::elementsSize, 1> imagePosition(reading.imagePosition.elements);
	e.block(0, 0, 4, 1) = imagePosition - desired_position;

	error = e;

	e_translation(0, 0) = e(0, 0);
	e_translation(1, 0) = e(1, 0);
	e_translation(2, 0) = e(2, 0);

	//		logDbg("ib_eih_visual_servo::get_position_change() %d, %d, %d, %+7.3lg\n", vsp_fradia->received_object.error.x, vsp_fradia->received_object.error.y, vsp_fradia->received_object.error.z, vsp_fradia->received_object.error.gamma);

	Eigen::Matrix <double, 6, 1> control;

	control = regulator->compute_control(e, dt);
	//logDbg("ib_eih_visual_servo::get_position_change() control: %+07.3lg, %+07.3lg, %+07.3lg\n", control(0, 0), control(1, 0), control(2, 0));

	Eigen::Matrix <double, 3, 1> camera_to_object_translation;
	camera_to_object_translation(0, 0) = control(0, 0);
	camera_to_object_translation(1, 0) = control(1, 0);
	camera_to_object_translation(2, 0) = control(2, 0);

	u_translation = e_T_c_position * camera_to_object_translation;

	//		logDbg("ib_eih_visual_servo::get_position_change() u_translation: %+07.3lg, %+07.3lg, %+07.3lg\n", u_translation(0, 0), u_translation(1, 0), u_translation(2, 0));

	u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));

	lib::Homog_matrix delta_position;
	delta_position.set_rotation_matrix(u_rotation);
	delta_position.set_translation_vector(u_translation);
	return delta_position;
}

bool ib_eih_visual_servo::is_object_visible_in_latest_reading()
{
	return reading.objectVisible;
}

void ib_eih_visual_servo::retrieve_reading()
{
	try {
		//		log_dbg("pb_visual_servo::retrieve_reading()\n");
		if (sensor->get_state() == discode_sensor::DSS_READING_RECEIVED) {
			//			log_dbg("pb_visual_servo::retrieve_reading(): sensor->get_state() == discode_sensor::DSS_READING_RECEIVED.\n");
			reading = sensor->retreive_reading <Types::Mrrocpp_Proxy::IBReading> ();
		}
	} catch (exception &ex) {
		log("pb_visual_servo::retrieve_reading(): %s\n", ex.what());
	}
}

}//namespace generator

}

}
