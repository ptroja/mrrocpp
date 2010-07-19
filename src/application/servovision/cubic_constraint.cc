/*
 * cubic_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include "cubic_constraint.h"
#include "lib/logger.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

cubic_constraint::cubic_constraint(const Eigen::Matrix <double, 3, 1>& p1, const Eigen::Matrix <double, 3, 1>& p2) :
	p1(p1), p2(p2)
{
	for (int i = 0; i < 3; ++i) {
		if (p1(i, 0) >= p2(i, 0)) {
			log("cubic_constraint::cubic_constraint(): p1(%d, 0) = %lg >= p2(%d, 0) = %lg\n", i, p1(i, 0), i, p2(i, 0));
		}
	}
}

cubic_constraint::~cubic_constraint()
{
}

void cubic_constraint::apply_constraint(lib::Homog_matrix& new_position)
{
	double t[3];
	new_position.get_translation_vector(t);
	for (int i = 0; i < 3; ++i) {
		t[i] = std::min(p2(i, 0), t[i]);
		t[i] = std::max(p1(i, 0), t[i]);
	}
	new_position.set_translation_vector(t);
}

bool cubic_constraint::is_position_ok(const lib::Homog_matrix& new_position)
{
	double t[3];
	new_position.get_translation_vector(t);
	for (int i = 0; i < 3; ++i) {
		if (t[i] < p1(i, 0) || p2(i, 0) < t[i]) {
			return false;
		}
	}
	return true;
}

} // namespace generator

}

}

}
