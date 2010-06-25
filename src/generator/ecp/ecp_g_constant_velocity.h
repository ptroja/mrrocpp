/*
 * ecp_g_constant_velocity.h
 *
 *  Created on: May 18, 2010
 *      Author: rtulwin
 */

#ifndef _ECP_G_CONSTANT_VELOCITY_H_
#define _ECP_G_CONSTANT_VELOCITY_H_

#include "generator/ecp/ecp_g_multiple_position.h"
#include "lib/mrmath/mrmath.h"
#include "lib/trajectory_pose/constant_velocity_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/constant_velocity_profile.h"
#include "generator/ecp/trajectory_interpolator/constant_velocity_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Generator which moves the robot with a constant velocity.
 */
class constant_velocity : public multiple_position<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose,
		ecp::common::generator::trajectory_interpolator::constant_velocity_interpolator,
		ecp::common::generator::velocity_profile_calculator::constant_velocity_profile> {
	public:
		/**
		 * Constructor. Sets the axes_num and pose_spec variables.
		 * @param axes_num number of axes for a given robot and representation
		 * @param pose_spec representation in which the robot position is expressed
		 */
		constant_velocity(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Loads a single trajectory pose described in joint coordinates to the list. Maximal velocities are set automatically.
		 */
		bool load_absolute_joint_trajectory_pose(vector<double> & coordinates);
		/**
		 * Performs calculation of the trajectory and interpolation. Fills in pose_vector and coordinate_vector.
		 * @return true if the calculation was succesfull
		 */
		bool calculate_interpolate();
		/**
		 * Destructor.
		 */
		virtual ~constant_velocity();
		/**
		 * Implementation of the first_step method.
		 */
		virtual bool first_step();
		/**
		 * Implementation of the next_step method.
		 */
		virtual bool next_step();

	private:

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_G_CONSTANT_VELOCITY_H_ */
