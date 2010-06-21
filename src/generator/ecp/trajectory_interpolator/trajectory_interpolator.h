/*
 * trajectory_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _TRAJECTORY_INTERPOLATOR_H_
#define _TRAJECTORY_INTERPOLATOR_H_

#include "lib/trajectory_pose/trajectory_pose.h"

#include <vector>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

class trajectory_interpolator {
public:
	trajectory_interpolator();
	virtual ~trajectory_interpolator();
	virtual bool interpolate(vector<ecp_mp::common::trajectory_pose::trajectory_pose>::iterator pose_vector_iterator, vector<vector<double> >::iterator coordinate_vector_iterator) = 0;
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _TRAJECTORY_INTERPOLATOR_H_ */
