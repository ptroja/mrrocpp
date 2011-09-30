/**
 * @file
 * @brief Contains declarations of the methods of constant_velocity_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _CONSTANT_VELOCITY_INTERPOLATOR_H_
#define _CONSTANT_VELOCITY_INTERPOLATOR_H_

#include "trajectory_interpolator.h"
#include "base/lib/trajectory_pose/constant_velocity_trajectory_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

/**
 * @brief Methods to perform the interpolation of the motion with the constant velocity.
 *
 * Class contains methods used to create the list of coordinates basing on the trajectory pose list which describes the motion of the robot with a constant velocity.
 *
 * @author rtulwin
 * @ingroup generators
 */
class constant_velocity_interpolator : public trajectory_interpolator<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose> {
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
	 * @return true if the interpolation was successful
	 */
	bool interpolate_relative_pose(std::vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, std::vector<std::vector<double> > & cv, const double mc);
	/**
	 * Method interpolates the absolute type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	bool interpolate_absolute_pose(std::vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, std::vector<std::vector<double> > & cv, const double mc);

private:
	/**
	 * Method generates a single relative type coordinate.
	 * @param node_counter number of current node (macrostep)
	 * @param it iterator to the list of positions
	 * @param axis_num number of current axis for which the calculations are performed
	 * @param mc time of a single macrostep
	 * @return single, generated coordinate
	 */
	double generate_relative_coordinate(int node_counter, std::vector <ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int axis_num, const double mc);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_INTERPOLATOR_H_ */
