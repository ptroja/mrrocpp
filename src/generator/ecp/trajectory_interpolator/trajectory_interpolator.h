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

/**
 * Base class for all trajectory interpolators.
 */
template <class Pos>
class trajectory_interpolator {
public:
	/**
	 * Constructor.
	 */
	trajectory_interpolator();
	/**
	 * Destructor.
	 */
	virtual ~trajectory_interpolator();
	/**
	 * Method interpolates the trajectory basing on the list of poses of type %constant_velocity_trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cit iterator to the list of coordinates
	 * @return true if the interpolation was successful
	 */
	virtual bool interpolate(typename vector<Pos>::iterator & pose_vector_iterator, vector<vector<double> >::iterator & coordinate_vector_iterator) = 0;

};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _TRAJECTORY_INTERPOLATOR_H_ */
