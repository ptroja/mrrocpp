/**
 * @file
 * @brief Contains declarations of the methods of constant_velocity_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _CONSTANT_VELOCITY_PROFILE_H_
#define _CONSTANT_VELOCITY_PROFILE_H_

#include "base/lib/trajectory_pose/constant_velocity_trajectory_pose.h"
#include "generator/lib/velocity_profile_calculator/velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

/**
 * @brief Class contains methods used to create and calculate the description of the motion of the robot with a constant velocity.
 *
 * This velocity profile calculator is used to fill in the trajectory pose list in a way that created trajectory describes the motion of the
 * robot with a constant velocity. Methods implemented here can cope with the absolute and relative type of motion.
 */
class constant_velocity_profile : public velocity_profile<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose> {
	public:
		/**
		 * Constructor.
		 */
		constant_velocity_profile();
		/**
		 * Destructor.
		 */
		virtual ~constant_velocity_profile();
		/**
		 * Calculates velocity for the given time and distance for a single axis in a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool calculate_constant_velocity(std::vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i);
		/**
		 * Calculates velocity for the given time and distance for a single pose.
		 * @param it iterator to the list of positions
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool calculate_constant_velocity_pose(std::vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it);
		/**
		 * Calculates time for the given velocity and distance for a single axis in a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the calculations are performed
		 * @return true if the time was calculated successfully (if all of the necessary information was provided)
		 */
		bool calculate_time(std::vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i);
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_PROFILE_H_ */
