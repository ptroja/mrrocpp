/*
 * cubic_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include <stdexcept>

#include "cubic_constraint.h"
#include "lib/logger.h"

using namespace logger;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

cubic_constraint::cubic_constraint(const lib::configurator& config, const std::string &section_name)
{
	translation_min = config.value <3, 1> ("translation_min", section_name);
	translation_max = config.value <3, 1> ("translation_max", section_name);

	cone_axis = config.value <3, 1> ("cone_axis", section_name);
	cone_axis = cone_axis / cone_axis.norm();

	min_inclination = config.value <double> ("min_inclination", section_name);

	if (!(0 < min_inclination || min_inclination < 0.9 * M_PI_2)) {
		throw runtime_error("min_inclination is wrong.");
	}

	wrist_rotation_min = config.value <double> ("min_wrist_rotation", section_name);
	wrist_rotation_max = config.value <double> ("max_wrist_rotation", section_name);
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
	lib::Xyz_Angle_Axis_vector position_aa;
	new_position.get_xyz_angle_axis(position_aa);

	Eigen::Matrix <double, 3, 1> axis = position_aa.block(3, 0, 3, 1);
	double wrist_rotation = axis.norm();

	if (axis(2, 0) < 0) {
		axis(0, 0) = -axis(0, 0);
		axis(1, 0) = -axis(1, 0);
		axis(2, 0) = -axis(2, 0);
		wrist_rotation = -wrist_rotation;
	}

	Eigen::Matrix <double, 2, 1> projected_axis = axis.block(0, 0, 2, 1);
	double inclination = atan2(axis(2, 0), projected_axis.norm());

	if (inclination < min_inclination) {
		return false;
	}

	if (!is_angle_between(wrist_rotation, wrist_rotation_min, wrist_rotation_max)) {
		return false;
	}

	return true;
}

double cubic_constraint::get_distance_from_allowed_area()
{
	return 1.0;
}

void cubic_constraint::apply_constraint()
{
	lib::Xyz_Angle_Axis_vector position_aa;
	new_position.get_xyz_angle_axis(position_aa);

	for (int i = 0; i < 3; ++i) {
		position_aa(i, 0) = min(translation_max(i, 0), position_aa(i, 0));
		position_aa(i, 0) = max(translation_min(i, 0), position_aa(i, 0));
	}

	Eigen::Matrix <double, 3, 1> axis = position_aa.block(3, 0, 3, 1);
	double wrist_rotation = axis.norm();

	if (axis(2, 0) < 0) {
		axis(0, 0) = -axis(0, 0);
		axis(1, 0) = -axis(1, 0);
		axis(2, 0) = -axis(2, 0);
		wrist_rotation = -wrist_rotation;
	}

	Eigen::Matrix <double, 2, 1> projected_axis = axis.block(0, 0, 2, 1);
	double inclination = atan2(axis(2, 0), projected_axis.norm());

	if (inclination < min_inclination) {
		inclination = min_inclination;

		axis(2, 0) = sin(inclination);
		axis.block(0, 0, 2, 1) = cos(inclination) * projected_axis / projected_axis.norm();
	}

	wrist_rotation = constrain_angle(wrist_rotation, wrist_rotation_min, wrist_rotation_max);

	position_aa.block(3, 0, 3, 1) = axis * wrist_rotation;
}

} // namespace generator

}

}

}
