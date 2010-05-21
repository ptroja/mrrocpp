/*
 * velocity_profile.h
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#ifndef _VELOCITY_PROFILE_H_
#define _VELOCITY_PROFILE_H_

#include <list>
#include <math.h>

#include "ecp_mp/trajectory_pose/trajectory_pose.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

class velocity_profile {
	public:
		/**
		 * Constructor.
		 */
		velocity_profile();
		/**
		 * Destructor.
		 */
		virtual ~velocity_profile();
		/**
		 * Method comparing two double values.
		 * @return true if values are the same
		 */
		bool eq(double a, double b);
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _VELOCITY_PROFILE_H_ */
