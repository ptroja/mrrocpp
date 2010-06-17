/*
 * constant_velocity_profile.h
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#ifndef _CONSTANT_VELOCITY_PROFILE_H_
#define _CONSTANT_VELOCITY_PROFILE_H_

#include "ecp_mp/trajectory_pose/constant_velocity_trajectory_pose.h"
#include "ecp/common/generator/velocity_profile_calculator/velocity_profile.h"

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
		 * Calculates velocity for the given time and distance.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool calculate_constant_velocity(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i);
		/**
		 * Calculates time for the given velocity and distance.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool calculate_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i);
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _CONSTANT_VELOCITY_PROFILE_H_ */
