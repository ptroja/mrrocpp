/**
 * @file
 * @brief Contains declarations of the methods of spline class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _ECP_G_SPLINE_H
#define _ECP_G_SPLINE_H

#include "ecp_mp_g_spline.h"
#include "generator/ecp/ecp_g_multiple_position.h"
#include "base/lib/trajectory_pose/spline_trajectory_pose.h"
#include "base/lib/datastr.h"
#include "generator/lib/velocity_profile_calculator/spline_profile.h"
#include "generator/lib/trajectory_interpolator/spline_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Generator which uses spline curves to calculate the trajectory. Polynomial generator.
 *
 * Generator can be used in three different modes - linear, cubic and quintic. User can freely choose between the different types of performed
 * calculation. Linear - motion with constant velocity. Cubic - continuous first derivative (continuous velocity). Quintic - continuous second
 * derivative (continuous acceleration).
 *
 * Usage: Load one or more of trajectory poses using one of the load methods. Velocities and accelerations are set automatically, however they can be
 * also set by the appropriate load methods. Call %calculate_interpolate() method.
 * If it returns true generator is ready to communicate with the robot. Call the %Move() method. The generator resets itself automatically after
 * successful termination of the assumed trajectory, however it is safe to call the %reset() method before the next use of the generator.
 *
 * @author rtulwin
 * @ingroup generators
 */
class spline : public multiple_position <ecp_mp::common::trajectory_pose::spline_trajectory_pose,
		ecp::common::generator::trajectory_interpolator::spline_interpolator,
		ecp::common::generator::velocity_profile_calculator::spline_profile>
{
public:
	/**
	 * Constructor. Sets the axes_num and pose_spec variables.
	 * @param _ecp_task current ecp task
	 * @param axes_num number of axes for a given robot and representation
	 * @param pose_spec representation in which the robot position is expressed
	 */
	spline(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);

	/**
	 * Destructor.
	 */
	virtual ~spline();
	/**
	 * Loads a single trajectory pose described in joint coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_absolute_joint_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in joint coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_relative_joint_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in motor coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_absolute_motor_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in motor coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_relative_motor_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in euler zyz coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_absolute_euler_zyz_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in euler zyz coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_relative_euler_zyz_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in angle axis coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_absolute_angle_axis_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads a single trajectory pose described in angle axis coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
	 * @param coordinates desired position
	 * @return true if the addition was successful
	 */
	bool load_relative_angle_axis_trajectory_pose(const std::vector <double> & coordinates);
	/**
	 * Loads the whole trajectory chain (possibly more than one position) stored in a text file.
	 * @param file_name name of the file with the trajectory
	 */
	bool load_trajectory_from_file(const char* file_name);
	/**
	 * Method load the absolute trajectory_pose object to the pose_vector.
	 * @param trajectory_pose pose to load
	 */
	bool load_absolute_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose);
	/**
	 * Method load the relative trajectory_pose object to the pose_vector.
	 * @param trajectory_pose pose to load
	 */
	bool load_relative_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose);
	/**
	 * Method sets the current interpolation type.
	 * @param type interpolation type
	 */
	void setType(splineType type);
private:
	/**
	 * Creates the vectors containing the information about the maximal and typical velocities for each representation.
	 * @param axes_num actual number of axes
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
	/**
	 * Prints single pose.
	 */
	void print_pose(const std::vector <ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it);
	/**
	 * Interpolation type: linear, cubic or quintic.
	 */
	splineType type;
	/**
	 * Loads trajectory pose.
	 * @return true if the addition was successful
	 */
	bool load_trajectory_pose(const std::vector <double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const std::vector <
			double> & v, const std::vector <double> & a, const std::vector <double> & v_max, const std::vector <double> & a_max);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // _ECP_G_SPLINE_H
