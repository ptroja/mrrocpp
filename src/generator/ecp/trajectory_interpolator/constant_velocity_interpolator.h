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

/**
 * @brief Methods to perform the interpolation of the motion with the constant velocity.
 *
 * Class contains methods used to create the list of coordinates basing on the trajectory pose list which describes the motion of the robot with a constant velocity.
 */
class constant_velocity_interpolator: public trajectory_interpolator<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose> {
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
	 * Method interpolates the relative type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was succesful
	 */
	bool interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double & mc);
	/**
	 * Method interpolates the absolute type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was succesful
	 */
	bool interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double & mc);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_INTERPOLATOR_H_ */
