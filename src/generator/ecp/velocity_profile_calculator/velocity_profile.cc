/*
 * velocity_profile.cc
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#include "velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

velocity_profile::velocity_profile() {
	// TODO Auto-generated constructor stub

}

velocity_profile::~velocity_profile() {
	// TODO Auto-generated destructor stub
}

bool velocity_profile::eq(double a, double b) {
	const double EPS = 0.0001;
	const double& diff = a - b;
	return diff < EPS && diff > -EPS;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
