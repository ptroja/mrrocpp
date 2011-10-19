/**
 * @file
 * @brief Contains declarations of the methods of spline_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _SPLINE_INTERPOLATOR_H
#define _SPLINE_INTERPOLATOR_H

#include "trajectory_interpolator.h"
#include "base/lib/trajectory_pose/spline_trajectory_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

/**
 * @brief Methods to perform the interpolation of the motion using spline curves.
 *
 * Class contains methods used to create the list of coordinates basing on the trajectory pose list which describes the motion of the robot using spline curves.
 *
 * @author rtulwin
 * @ingroup generators
 */
class spline_interpolator : public trajectory_interpolator<ecp_mp::common::trajectory_pose::spline_trajectory_pose> {
public:
        /**
         * Constructor.
         */
        spline_interpolator();
        /**
         * Destructor.
         */
        virtual ~spline_interpolator();
	/**
	 * Method interpolates the relative type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
        bool interpolate_relative_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<std::vector<double> > & cv, const double mc);
	/**
	 * Method interpolates the absolute type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
	 * @param it iterator to the list of positions
	 * @param cv list of coordinates
	 * @param mc time of a single macrostep
	 * @return true if the interpolation was successful
	 */
	bool interpolate_absolute_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<std::vector<double> > & cv, const double mc);

private:
        /**
         * Method generates consecutive powers of a number.
         * @param x number to be powered
         * @param power maximal power
         * @param powers vector to be filled in with powers x
         */
        inline void generatePowers(int power, double x, double * powers);
        /**
         * Method calculates absolute position at the given time.
         * @param it iterator to the list of positions
         * @param i number of current axis
         * @param time given time for which position is desired
         */
        double calculate_position(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time);
        /**
         * Method calculates valocity at the given time.
         * @param it iterator to the list of positions
         * @param i number of current axis
         * @param time given time for which velocity is desired
         */
        double calculate_velocity(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time);
        /**
         * Method calculates acceleration at the given time.
         * @param it iterator to the list of positions
         * @param i number of current axis
         * @param time given time for which acceleration is desired
         */
        double calculate_acceleration(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time);
        /**
         * Method generates a single relative type coordinate.
         * @param node_counter number of current node (macrostep)
         * @param it iterator to the list of positions
         * @param axis_num number of current axis for which the calculations are performed
         * @param mc time of a single macrostep
         * @return single, generated coordinate
         */
        double generate_relative_coordinate(int node_counter, std::vector <ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int axis_num, const double mc);
};

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // _SPLINE_INTERPOLATOR_H
