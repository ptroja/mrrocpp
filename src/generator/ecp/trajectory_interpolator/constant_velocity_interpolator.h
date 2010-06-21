/*
 * constant_velocity_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _CONSTANT_VELOCITY_INTERPOLATOR_H_
#define _CONSTANT_VELOCITY_INTERPOLATOR_H_

#include "trajectory_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

class constant_velocity_interpolator: public mrrocpp::ecp::common::generator::trajectory_interpolator::trajectory_interpolator {
public:
	constant_velocity_interpolator();
	virtual ~constant_velocity_interpolator();

};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_INTERPOLATOR_H_ */
