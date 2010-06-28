/*
 * constant_velocity_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _CONSTANT_VELOCITY_INTERPOLATOR_H_
#define _CONSTANT_VELOCITY_INTERPOLATOR_H_

#include "trajectory_interpolator.h"
#include "lib/trajectory_pose/constant_velocity_trajectory_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

class constant_velocity_interpolator: public mrrocpp::ecp::common::generator::trajectory_interpolator::trajectory_interpolator<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose> {
public:
	/**
	 * Constructor.
	 */
	constant_velocity_interpolator();
	/**
	 * Destructor.
	 */
	virtual ~constant_velocity_interpolator();
	/**
	 * Method interpolates the trajectory basing on the list of poses of type %constant_velocity_trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cit iterator to the list of coordinates
	 * @return true if the interpolation was succesful
	 */
	bool interpolate(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> >::iterator & cit);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_INTERPOLATOR_H_ */
