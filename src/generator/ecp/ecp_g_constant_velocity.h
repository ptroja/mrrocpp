/**
 * @file
 * @brief Contains declarations of the methods of constant_velocity class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _ECP_G_CONSTANT_VELOCITY_H_
#define _ECP_G_CONSTANT_VELOCITY_H_

#include "generator/ecp/ecp_g_multiple_position.h"
#include "base/lib/trajectory_pose/constant_velocity_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/constant_velocity_profile.h"
#include "generator/ecp/trajectory_interpolator/constant_velocity_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Generator which moves the robot with a constant velocity.
 *
 * Generator is able to generate the list of trajectory points and send it to the robot. Trajectory is generated in a way that
 * the robot moves with a constant velocity. The generator is able to interpolate the trajectory described as the list of
 * %constant_velocity_trajectory_pose objects.
 *
 * Usage:
 * Load one or more of trajectory poses using one of the load methods. Velocities are set automatically. Call %calculate_interpolate() method.
 * If it returns true generator is ready to communicate with the robot. Call the %Move() method. The generator resets itself automatically after
 * successful termination of the assumed trajectory, however it is safe to call the %reset() method before the next use of the generator.
 *
 * @author rtulwin
 * @ingroup generators
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
		 * Loads a single trajectory pose described in joint coordinates (absolute motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_joint_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in joint coordinates (relative motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_joint_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in motor coordinates (absolute motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_motor_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in motor coordinates (relative motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_motor_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in euler zyz coordinates (absolute motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_euler_zyz_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in euler zyz coordinates (relative motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_euler_zyz_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in angle axis coordinates (absolute motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_angle_axis_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in angle axis coordinates (relative motion) to the list. Maximal velocities are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_angle_axis_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Destructor.
		 */
		virtual ~constant_velocity();

	private:
		/**
		 * Loads trajectory pose.
		 * @return true if the addition was successful
		 */
		bool load_trajectory_pose(const std::vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const std::vector<double> & v, const std::vector<double> & v_max);
		/**
		 * Creates the vectors containing the information about the maximal and typical velocities for each representation.
		 * @axes_num actual number of axes
		 */
		void create_velocity_vectors(int axes_num);
		/**
		 * Calculates trajectory.
		 * @return true if calculation was successful.
		 */
		bool calculate();
		/**
		 * Method used to print list of positions.
		 */
		void print_pose_vector();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_G_CONSTANT_VELOCITY_H_ */
