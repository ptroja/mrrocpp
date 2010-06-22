/*
 * cubic_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include "cubic_constraint.h"
#include "lib/logger.h"

using namespace logger;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

cubic_constraint::cubic_constraint(const Eigen::Matrix <double, 3, 1>& translation_min, const Eigen::Matrix <double, 3,
		1>& translation_max, const Eigen::Matrix <double, 3, 1>& rotation_min, const Eigen::Matrix <double, 3, 1>& rotation_max) :
	translation_min(translation_min), translation_max(translation_max), rotation_min(rotation_min),
			rotation_max(rotation_max)
{
	//	for (int i = 0; i < 3; ++i) {
	//		if (p1(i, 0) >= p2(i, 0)) {
	//			log("cubic_constraint::cubic_constraint(): p1(%d, 0) = %lg >= p2(%d, 0) = %lg\n", i, p1(i, 0), i, p2(i, 0));
	//		}
	//	}
}

cubic_constraint::~cubic_constraint()
{
}

void cubic_constraint::apply_constraint(lib::Homog_matrix& new_position)
{
	lib::Xyz_Angle_Axis_vector position_aa;
	new_position.get_xyz_angle_axis(position_aa);

	for (int i = 0; i < 3; ++i) {
		position_aa(i, 0) = min(translation_max(i, 0), position_aa(i, 0));
		position_aa(i, 0) = max(translation_min(i, 0), position_aa(i, 0));

		if (rotation_min(i, 0) <= rotation_max(i, 0)) {
//			position_aa(i + 3, 0) = min(rotation_max(i, 0), position_aa(i + 3, 0));
//			position_aa(i + 3, 0) = max(rotation_min(i, 0), position_aa(i + 3, 0));
		} else {
			position_aa(i + 3, 0) = max(rotation_max(i, 0), position_aa(i + 3, 0));
			position_aa(i + 3, 0) = min(rotation_min(i, 0), position_aa(i + 3, 0));
		}
	}
	new_position.set_from_xyz_angle_axis(position_aa);
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
}

double cubic_constraint::get_distance_from_allowed_area()
{
}

void cubic_constraint::apply_constraint()
{
}

} // namespace generator

}

}

}
