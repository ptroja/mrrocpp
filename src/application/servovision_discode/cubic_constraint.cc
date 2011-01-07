/*
 * cubic_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <iostream>

#include "cubic_constraint.h"
#include "base/lib/logger.h"

using namespace logger;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace servovision {

cubic_constraint::cubic_constraint(const lib::configurator& config, const std::string &section_name)
{
	cube_position = config.value <3, 4> ("cube_position", section_name);
	cube_size = config.value <3, 1> ("cube_size", section_name);

	spherical_cone_rotation = config.value <3, 4> ("spherical_cone_rotation", section_name);
	spherical_cone_rotation(0, 3) = spherical_cone_rotation(1, 3) = spherical_cone_rotation(2, 3) = 0;

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

lib::Homog_matrix cubic_constraint::apply_constraint(const lib::Homog_matrix& current_position)
{
	lib::Homog_matrix constrained_position;

	constrained_position = (!cube_position) * current_position;

//	cout<<"constrained_position 1:\n"<<constrained_position<<"\n";
//	cout.flush();

	// constrain translation
	for (int i = 0; i < 3; ++i) {
		constrained_position(i, 3) = min(cube_size(i, 0) / 2, constrained_position(i, 3));
		constrained_position(i, 3) = max(-cube_size(i, 0) / 2, constrained_position(i, 3));
	}

	constrained_position = cube_position * constrained_position;

	constrained_position = constrain_rotation(constrained_position);

	return constrained_position;
}

lib::Homog_matrix cubic_constraint::constrain_rotation(const lib::Homog_matrix& current_position)
{
	// calculate difference between actual rotation and allowed rotation
	lib::Homog_matrix rot = (!spherical_cone_rotation) * current_position;

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
	lib::Homog_matrix position_with_constrained_rotation(spherical_cone_rotation * rot);
	return position_with_constrained_rotation;
}

} // namespace generator

}

}
