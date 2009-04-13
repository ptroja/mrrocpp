#include <math.h>

#include "lib/com_buf.h"

void playerpos_goal_t::setGoal(double _x, double _y, double _t)
{
	x = _x;
	y = _y;
	t = _t;
}

void playerpos_goal_t::turn(double angle)
{
	t += 1.42*angle; //friction correction
}

void playerpos_goal_t::forward(double length)
{
	x += length*cos(t);
	y += length*sin(t);
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

playerpos_goal_t::playerpos_goal_t() :
	x(0.0), y(0.0), t(0.0)
{
}

playerpos_goal_t::playerpos_goal_t(double _x, double _y, double _t) :
	x(_x), y(_y), t(_t)
{
}
