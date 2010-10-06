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

namespace servovision {

pb_eih_visual_servo::pb_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	pb_visual_servo(regulator, section_name, configurator)
{
	E_T_C = configurator.value <3, 4> ("E_T_C", section_name);
}

pb_eih_visual_servo::~pb_eih_visual_servo()
{
}

lib::Homog_matrix pb_eih_visual_servo::compute_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix C_T_G(vsp_fradia->get_reading_message().position);
	lib::Homog_matrix error_matrix;

	error_matrix = G_T_E_desired * E_T_C * C_T_G;

	lib::Xyz_Angle_Axis_vector aa_vector;
	error_matrix.get_xyz_angle_axis(aa_vector);
	error = aa_vector;
	aa_vector = regulator->compute_control(aa_vector, dt);

	lib::Homog_matrix delta_position;
	delta_position.set_from_xyz_angle_axis(aa_vector);
	return delta_position;
}

} // namespace generator

}

}

