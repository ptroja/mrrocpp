/*
 * trajectory_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _TRAJECTORY_INTERPOLATOR_H_
#define _TRAJECTORY_INTERPOLATOR_H_

#include "base/lib/trajectory_pose/trajectory_pose.h"
#include "base/lib/mrmath/mrmath.h"
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
	trajectory_interpolator() {

	}
	/**
	 * Destructor.
	 */
	virtual ~trajectory_interpolator() {

	}
	/**
	 * Method interpolates the relative type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param coordinate_vector list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	virtual bool interpolate_relative_pose(typename std::vector<Pos>::iterator & pose_vector_iterator, std::vector<std::vector<double> > & coordinate_vector, const double mc) = 0;
	/**
	 * Method interpolates the absolute type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param coordinate_vector list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	virtual bool interpolate_absolute_pose(typename std::vector<Pos>::iterator & pose_vector_iterator, std::vector<std::vector<double> > & coordinate_vector, const double mc) = 0;
	/**
	 * Method is used to interpolate the Angle Axis absolute pose, which was previously transformed into relative pose using the velocity_profile::calculate_relative_angle_axis_vector method (coordinates vector is now a relative vector).
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	virtual bool interpolate_angle_axis_absolute_pose_transformed_into_relative(typename std::vector <Pos>::iterator & pose_vector_iterator, std::vector <std::vector <double> > & cv, const double mc) = 0;

};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _TRAJECTORY_INTERPOLATOR_H_ */
