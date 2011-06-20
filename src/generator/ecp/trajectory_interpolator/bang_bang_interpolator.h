/**
 * @file
 * @brief Contains declarations of the methods of bang_bang_interpolator class.
 * @author rtulwin
 * @ingroup generators
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
 * Class contains methods used to create the list of coordinates basing on the trajectory pose list which describes the motion of the robot with a trapezoidal velocity profile.
 *
 * @author rtulwin
 * @ingroup generators
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
	/**
	 * Method generates a single relative type coordinate.
	 * @param node_counter number of current node (macrostep)
	 * @param it iterator to the list of positions
	 * @param axis_num number of current axis for which the calculations are performed
	 * @param mc time of a single macrostep
	 * @return single, generated coordinate
	 */
	double generate_relative_coordinate(int node_counter, std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int axis_num, const double mc);
	/**
	 * Method generates single coordinate, relative or absolute type.
	 * @param node_counter number of current node (macrostep), here it is usually necessary to pass node_counter+1
	 * @param interpolation_node_no number of macrosteps in pose
	 * @param mc time of a single macrostep
	 * @param start_position initial position (of the segment)
	 * @param v_p initial velocity (of the segment)
	 * @param v_r velocity of the uniform motion phase
	 * @param v_k terminal velocity (of the segment)
	 * @param a_r acceleration
	 * @param k direction (1 or -1)
	 * @param acc macrostep in which the acceleration phase ends
	 * @param uni macrostep in which the second (uniform motion) phase ends
	 * @param s_acc distance covered in the acceleration phase
	 * @param s_uni distance covered in the uniform motion phase
	 * @param motion_type type of the commanded motion (ABSOLUTE or RELATIVE)
	 */
	double generate_next_coordinate(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r, double v_k, double a_r, double k, double acc, double uni, double s_acc, double s_uni, lib::MOTION_TYPE motion_type, const double mc);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _BANG_BANG_INTERPOLATOR_H_ */
