/*
 * trajectory_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _TRAJECTORY_INTERPOLATOR_H_
#define _TRAJECTORY_INTERPOLATOR_H_

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

class trajectory_interpolator {
public:
	trajectory_interpolator();
	virtual ~trajectory_interpolator();
	//virtual bool calculate_interpolate();
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _TRAJECTORY_INTERPOLATOR_H_ */
