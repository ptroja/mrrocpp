/**
 * @file
 * @brief Contains declarations of the methods of spline_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _SPLINE_PROFILE_H
#define _SPLINE_PROFILE_H

#include "base/lib/trajectory_pose/spline_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

/**
 * @brief Class contains methods used to create and calculate the description of the motion of the robot using spline curves.
 *
 * This velocity profile calculator is used to fill in the trajectory pose list in a way that created trajectory describes the motion of the
 * robot using spline curves. Methods implemented here can cope with the absolute and relative type of motion.
 */
class spline_profile : public velocity_profile<ecp_mp::common::trajectory_pose::spline_trajectory_pose> {
        public:
                /**
                 * Constructor.
                 */
                spline_profile();
                /**
                 * Destructor.
                 */
                virtual ~spline_profile();
                /**
                 * Calculates time for the given velocity and distance for a single axis in a single pose.
                 * @param it iterator to the list of positions
                 * @param i number of axis for which the calculations are performed
                 * @return true if the time was calculated successfully (if all of the necessary information was provided)
                 */
                bool calculate_time(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i);
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // _SPLINE_PROFILE_H
