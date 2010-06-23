/*
 * position_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include "position_constraint.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

position_constraint::position_constraint()
{
}

position_constraint::~position_constraint()
{
}

void position_constraint::set_new_position(const lib::Homog_matrix& new_position)
{
	this->new_position = new_position;
}
const lib::Homog_matrix& position_constraint::get_constrained_position()
{
	return new_position;
}

double position_constraint::normalize_angle(double angle)
{
	while (angle >= M_PI)
		angle -= 2 * M_PI;
	while (angle < -M_PI)
		angle += 2 * M_PI;
	return angle;
}

bool position_constraint::is_angle_between(double angle, double min, double max)
{
	angle = normalize_angle(angle);
	min = normalize_angle(min);
	max = normalize_angle(max);
	if (min <= max) {
		return min <= angle && angle <= max;
	} else { // min > max
		return min <= angle || angle <= max;
	}
}

} // namespace generator

}

}

}
