/**
 * @file constant_velocity_trajectory_pose.h
 * @brief Header file for constant_velocity_trajectory_pose
 *
 * Contains declaration of bang_bang_trajectory_pose class and its methods.
 */

#if !defined(_ECP_CONSTANT_VELOCITY_TRAJECTORY_POSE_H)
#define  _ECP_CONSTANT_VELOCITY_TRAJECTORY_POSE_H

#include "base/lib/trajectory_pose/trajectory_pose.h"
#include <vector>

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

/**
 * Class is a container used by generators performing motion with the constant velocity.
 * One instance of the class contains information about one segment of the trajectory (single movement in one particular direction).
 */
class constant_velocity_trajectory_pose : public trajectory_pose {
public:
  /**
   * Maximal velocity for the movement, for each axis. Percent of the v_max value. Should be a value between 0 - 1.
   */
  std::vector<double> v;
  /**
   * Maximal velocity set for the given robot (v_r = v * v_max).
   */
  std::vector<double> v_max;
  /**
   * Initial position for the pose.
   */
  std::vector<double> start_position;
  /**
   * Maximal velocity for the given segment (pose) (calculated, can be smaller or equal to v).
   */
  std::vector<double> v_r;
  /**
   * Empty constructor.
   */
  constant_velocity_trajectory_pose (void);
  /**
   * Constructor which initiates some variables (those which can be found in the file containing trajectory).
   * @param at representation used in the given pose
   * @param coordinates desired position for all of the axes
   * @param v maximal velocities for the trajectory segment for all of the axes
   */
  constant_velocity_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		  const std::vector<double> & coordinates,
		  const std::vector<double> & v);
  /**
   * Destructor.
   */
  ~constant_velocity_trajectory_pose();

  //bool set_max_velocity(const std::vector<double> & v);

};

} // namespace trajectory_pose
} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_CONSTANT_VELOCITY_TRAJECTORY_POSE_H */
