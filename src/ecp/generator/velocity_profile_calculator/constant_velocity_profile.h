/*
 * constant_velocity_profile.h
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#ifndef _CONSTANT_VELOCITY_PROFILE_H_
#define _CONSTANT_VELOCITY_PROFILE_H_

#include "ecp_mp/trajectory_pose/constant_velocity_trajectory_pose.h"
#include "ecp/generator/velocity_profile_calculator/velocity_profile.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

class constant_velocity_profile : public velocity_profile {
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
		bool calculate_constant_velocity(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i);
		/**
		 * Calculates velocity for the given time and distance for a single pose.
		 * @param it iterator to the list of positions
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool calculate_constant_velocity_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it);
		/**
		 * Calculates time for the given velocity and distance for a single axis in a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the calculations are performed
		 * @return true if the time was calculated successfully (if all of the necessary information was provided)
		 */
		bool calculate_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i);
		/**
		 * Calculates time for the given velocity and distance for all axes in a single pose.
		 * @param it iterator to the list of positions
		 * @return true if the time was calculated successfully (if all of the necessary information was provided)
		 */
		bool calculate_time_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it);
		/**
		 * Calculates distance for all of the axes in a single trajectory pose and sets the directions of movements.
		 * @param it iterator to the list of positions
		 * @return true if the set of the distance and direction was successful (usually is if the vectors start_position and coordinates were initiated and filled in before)
		 */
		bool calculate_distance_direction_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it);
		/**
		 * Calculates the longest time from the times vector and stores it in t variable in the pose.
		 * @param it iterator to the list of positions
		 * @return true if the set of the time was successful (usually is if the vector times was initiated and filled in before)
		 */
		bool calculate_pose_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it);
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_PROFILE_H_ */
