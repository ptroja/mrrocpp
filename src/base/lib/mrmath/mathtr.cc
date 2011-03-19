/**
 * \file mathtr.cc
 *
 * \brief Angle range reduction routines
 *
 * \author Tomasz Kornuta <tkornuta@gmail.com>
 */

#include <cmath>

#include "base/lib/mrmath/mathtr.h"

namespace mrrocpp {
namespace lib {

double reduce(double angle)
{
	while (angle > M_PI)
		angle -= 2 * M_PI;

	while (angle <= -M_PI)
		angle += 2 * M_PI;

	return (angle);
}

double reduce(double angle, double min, double max, double offset)
{
	while (angle >= max)
		angle -= offset;

	while (angle < min)
		angle += offset;

	return (angle);
}

} // namespace lib
} // namespace mrrocpp

