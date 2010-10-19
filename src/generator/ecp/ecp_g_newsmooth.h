/**
 * @file
 * @brief Contains declarations of the methods of newsmooth class.
 * @author rtulwin
 * @ingroup generators
 */

#if !defined(_ECP_GEN_NEWSMOOTH_H)
# define _ECP_GEN_NEWSMOOTH_H

#include "generator/ecp/ecp_g_multiple_position.h"
#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/bang_bang_profile.h"
#include "generator/ecp/trajectory_interpolator/bang_bang_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Smooth trajectory generator which has an ability to calculate every trajectory (posiada moce super krowy).
 *
 * Usage:
 * Load one or more of trajectory poses using one of the load methods. Velocities and accelerations are set automatically. Call %calculate_interpolate() method.
 * If it returns true generator is ready to communicate with the robot. Call the %Move() method. The generator resets itself automatically after
 * successful termination of the assumed trajectory, however it is safe to call the %reset() method before the next use of the generator.
 *
 * @author rtulwin
 * @ingroup generators
 */
class newsmooth : public multiple_position<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose,
ecp::common::generator::trajectory_interpolator::bang_bang_interpolator,
ecp::common::generator::velocity_profile_calculator::bang_bang_profile> {

	private:
		/**
		 * Creates the vectors containning the information about the maximal and typical velocities and accelerations for each representation.
		 * @param axes_num actual number of axes
		 */
		void create_velocity_vectors(int axes_num);
		/**
		 * Calculates trajectory.
		 * @return true if calculation was successful.
		 */
		bool calculate();
		/**
		 * Loads trajectory pose.
		 * @return true if the addition was successful
		 */
		bool load_trajectory_pose(const std::vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const std::vector<double> & v, const std::vector<double> & a, const std::vector<double> & v_max, const std::vector<double> & a_max);
		/**
		 * Method used to print list of positions.
		 */
		void print_pose_vector();
		/**
		 * Prints single pose.
		 */
		void print_pose(std::vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);

	public:
		/**
		 * Constructor.
		 */
		newsmooth(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Destructor.
		 */
		virtual ~newsmooth();
		/**
		 * Loads a single trajectory pose described in joint coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_joint_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in joint coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_joint_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in motor coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_motor_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in motor coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_motor_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in euler zyz coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_euler_zyz_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in euler zyz coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_euler_zyz_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in angle axis coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_angle_axis_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in angle axis coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_angle_axis_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads the whole trajectory chain (possibly more than one position) stored in a text file.
		 * @param file_name name of the file with the trajectory
		 */
		bool load_trajectory_from_file(const char* file_name);

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
