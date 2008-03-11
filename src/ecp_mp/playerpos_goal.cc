#include <math.h>

#include "common/com_buf.h"

void playerpos_goal_t::setGoal(double _x, double _y, double _t)
{
	x = _x;
	y = _y;
	t = _t;
}

void playerpos_goal_t::turn(double angle)
{
	t += angle;
}

void playerpos_goal_t::forward(double length)
{
	x += length*std::cos(t);
	y += length*std::sin(t);
}

double playerpos_goal_t::getX() const
{
	return x;
}

double playerpos_goal_t::getY() const
{
	return y;
}

double playerpos_goal_t::getT() const
{
	return t;
}