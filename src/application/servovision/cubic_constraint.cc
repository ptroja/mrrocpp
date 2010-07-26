/*
 * cubic_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <iostream>

#include "cubic_constraint.h"
#include "lib/logger.h"

using namespace logger;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace servovision {

cubic_constraint::cubic_constraint(const lib::configurator& config, const std::string &section_name)
{
	translation_min = config.value <3, 1> ("translation_min", section_name);
	translation_max = config.value <3, 1> ("translation_max", section_name);

	//Eigen::Matrix <double, 3, 3> cone_rotation_e = config.value <3, 1> ("cone_rotation", section_name);
	//	cone_axis = cone_axis / cone_axis.norm();

	cone_rotation = config.value <3, 4> ("cone_rotation", section_name);
	cone_rotation(0, 3) = cone_rotation(1, 3) = cone_rotation(2, 3) = 0;

	min_inclination = config.value <double> ("min_inclination", section_name);

	if (!(0 < min_inclination || min_inclination < 0.9 * M_PI_2)) {
		throw runtime_error("min_inclination is wrong.");
	}

	wrist_rotation_min = config.value <double> ("wrist_rotation_min", section_name);
	wrist_rotation_max = config.value <double> ("wrist_rotation_max", section_name);
}

cubic_constraint::~cubic_constraint()
{
}

bool cubic_constraint::is_translation_ok()
{
	double t[3];
	new_position.get_translation_vector(t);
	for (int i = 0; i < 3; ++i) {
		if (t[i] < translation_min(i, 0) || translation_max(i, 0) < t[i]) {
			return false;
		}
	}
	return true;
}

bool cubic_constraint::is_rotation_ok()
{
	log("cubic_constraint::is_rotation_ok()\n");

	lib::Homog_matrix rot = (!cone_rotation) * new_position;

	lib::Xyz_Angle_Axis_vector position_aa;
	rot.get_xyz_angle_axis(position_aa);

	Eigen::Matrix <double, 3, 1> axis = position_aa.block(3, 0, 3, 1);
	double wrist_rotation = axis.norm();
	axis = axis / wrist_rotation;

	if (axis(2, 0) < 0) {
		axis(0, 0) = -axis(0, 0);
		axis(1, 0) = -axis(1, 0);
		axis(2, 0) = -axis(2, 0);
		wrist_rotation = -wrist_rotation;
	}

	Eigen::Matrix <double, 2, 1> projected_axis = axis.block(0, 0, 2, 1);
	double inclination = atan2(axis(2, 0), projected_axis.norm());
	cout << "axis = " << axis << endl;
	cout << "projected_axis = " << projected_axis << endl;
	cout << "inclination = " << inclination << endl;
	cout.flush();

	if (inclination < min_inclination) {
		log("inclination < min_inclination\n");
		return false;
	}

	if (!is_angle_between(wrist_rotation, wrist_rotation_min, wrist_rotation_max)) {
		log("!is_angle_between\n");
		return false;
	}
	log("rotation_ok()\n");
	return true;
}

double cubic_constraint::get_distance_from_allowed_area()
{
	return 1.0;
}

void cubic_constraint::apply_constraint()
{
	// constrain translation
	for (int i = 0; i < 3; ++i) {
		new_position(i, 3) = min(translation_max(i, 0), new_position(i, 3));
		new_position(i, 3) = max(translation_min(i, 0), new_position(i, 3));
	}

	// calculate difference between actual rotation and allowed rotation
	lib::Homog_matrix rot = (!cone_rotation) * new_position;

	lib::Xyz_Angle_Axis_vector position_aa;
	rot.get_xyz_angle_axis(position_aa);

	// get AA representation, where Z coordinate is always non-negative
	Eigen::Matrix <double, 3, 1> axis = position_aa.block(3, 0, 3, 1);
	double wrist_rotation = axis.norm();
	axis = axis / wrist_rotation;

	if (axis(2, 0) < 0) {
		axis(0, 0) = -axis(0, 0);
		axis(1, 0) = -axis(1, 0);
		axis(2, 0) = -axis(2, 0);
		wrist_rotation = -wrist_rotation;
	}

	// project rotation versor on XY plane
	Eigen::Matrix <double, 2, 1> projected_axis = axis.block(0, 0, 2, 1);
	// calculate inclination - angle between rotation versor and XY plane
	double inclination = atan2(axis(2, 0), projected_axis.norm());
	// constrain inclination
	if (inclination < min_inclination) {
		inclination = min_inclination;

		axis(2, 0) = sin(inclination);
		axis.block(0, 0, 2, 1) = cos(inclination) * projected_axis / projected_axis.norm();
	}
	// constrain wrist rotation
	wrist_rotation = constrain_angle(wrist_rotation, wrist_rotation_min, wrist_rotation_max);
	// get back from AA to rotation matrix
	position_aa.block(3, 0, 3, 1) = axis * wrist_rotation;
	rot.set_from_xyz_angle_axis(position_aa);
	// get back to rotation relative to robot's base
	new_position.set_rotation_matrix(cone_rotation * rot);
}

} // namespace generator

}

}
