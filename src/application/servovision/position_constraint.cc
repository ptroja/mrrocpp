/*
 * position_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include "position_constraint.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

position_constraint::position_constraint()
{
}

position_constraint::~position_constraint()
{
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

double position_constraint::constrain_angle(double angle, double min, double max)
{
	double division = (min + max) / 2;

	if (min <= max) {
		if (division <= 0) {
			division += M_PI;
		} else {
			division -= M_PI;
		}
	}

	if (is_angle_between(angle, division, min)) {
		return min;
	}
	if (is_angle_between(angle, max, division)) {
		return max;
	}
	return angle;
}

} // namespace generator
}
}
