/*
 * bang_bang_interpolator.h
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#ifndef _BANG_BANG_INTERPOLATOR_H_
#define _BANG_BANG_INTERPOLATOR_H_

#include "base/lib/com_buf.h"
#include "trajectory_interpolator.h"
#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

/**
 * @brief Methods to perform the interpolation of the motion with the trapezoidal velocity profile.
 *
 * Class contains methods used to create the list of coordinates basing on the trajectory pose list which describes the motion of the robot with a trapezoidal velocity.
 */
class bang_bang_interpolator : public mrrocpp::ecp::common::generator::trajectory_interpolator::trajectory_interpolator <
		ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>
{
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
	 * Method interpolates the relative type trajectory basing on the list of poses stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	bool interpolate_relative_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, std::vector <std::vector <double> > & cv, const double mc);
	/**
	 * Method interpolates the absolute type trajectory basing on the list of poses stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	bool interpolate_absolute_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, std::vector <std::vector <double> > & cv, const double mc);

private:
	double generate_next_coords(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r, double v_k, double a_r, double k, double przysp, double jedn, double s_przysp, double s_jedn, lib::MOTION_TYPE type);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _BANG_BANG_INTERPOLATOR_H_ */
