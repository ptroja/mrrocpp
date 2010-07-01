/*
 * bang_bang_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _BANG_BANG_INTERPOLATOR_H_
#define _BANG_BANG_INTERPOLATOR_H_

#include "trajectory_interpolator.h"
#include "lib/trajectory_pose/bang_bang_trajectory_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

/**
 * @brief Methods to perform the interpolation of the motion with the trapezoidal velocity.
 *
 * Class contains methods used to create the list of coordinates basing on the trajectory pose list which describes the motion of the robot with a trapezoidal velocity.
 */
class bang_bang_interpolator: public mrrocpp::ecp::common::generator::trajectory_interpolator::trajectory_interpolator<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose> {
public:
	/**
	 * Constructor.
	 */
	bang_bang_interpolator();
	/**
	 * Destructor.
	 */
	virtual ~bang_bang_interpolator();
	/**
	 * Method interpolates the relative type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was succesful
	 */
	bool interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double & mc);
	/**
	 * Method interpolates the absolute type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was succesful
	 */
	bool interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double & mc);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _BANG_BANG_INTERPOLATOR_H_ */
