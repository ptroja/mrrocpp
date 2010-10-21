/**
 * @file
 * @brief Contains declarations of the methods of bang_bang_trajectory_pose class.
 * @author rtulwin
 * @ingroup generators
 */

#if !defined(_ECP_BANG_BANG_TRAJECTORY_POSE_H)
#define  _ECP_BANG_BANG_TRAJECTORY_POSE_H

#include "base/lib/trajectory_pose/trajectory_pose.h"
#include <vector>

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

/**
 * @brief Class is a container used by smooth trajectory generator.
 *
 * One instance of the class contains information about one segment of the trajectory (single movement in one particular direction).
 *
 * @author rtulwin
 * @ingroup trajectory_pose
 */
class bang_bang_trajectory_pose : public trajectory_pose {
public:
  /**
   * Initial velocity for the pose, for each axis.
   */
  std::vector<double> v_p;
  /**
   * Final velocity for the pose, for each axis.
   */
  std::vector<double> v_k;
  /**
   * Maximal velocity for the movement, for each axis. Percent of the v_max value. Should be a value between 0 - 1.
   */
  std::vector<double> v;
  /**
   * Maximal velocity set for the given robot (v_r = v * v_max).
   */
  std::vector<double> v_max;
  /**
   * Maximal acceleration for the motion, for each axis.
   */
  std::vector<double> a;
  /**
   * Maximal acceleration set for the given robot (a_r = a * a_max).
   */
  std::vector<double> a_max;
  /**
   * Number of the macrostep in which the first part of the movement ends (first out of three). Duration of the first part of the motion in macrosteps.
   */
  std::vector<double> acc;
  /**
   * Duration of the second part of the motion in macrosteps.
   */
  std::vector<double> uni;
  /**
   * Distance covered in the second part of the movement.
   */
  std::vector<double> s_uni;
  /**
   * Distance covered in the first part of the movement.
   */
  std::vector<double> s_acc;
  /**
   * Distance covered in the third part of the movement.
   */
  std::vector<double> s_dec;
  /**
   * Initial position for the pose.
   */
  std::vector<double> start_position;
  /**
   * Maximal acceleration for the given segment (pose) (calculated, can be smaller or equal to a).
   */
  std::vector<double> a_r;
  /**
   * Maximal velocity for the given segment (pose) (calculated, can be smaller or equal to v).
   */
  std::vector<double> v_r;
  /**
   * Motion kinematic_model_with_tool.
   */
  std::vector<int> model;

  /**
   * Empty constructor.
   */
  bang_bang_trajectory_pose (void);
  /**
   * Constructor which initiates some variables (those which can be found in the file containing trajectory).
   * @param arm_type representation used in the given pose
   * @param coordinates desired position for all of the axes
   * @param v maximal velocities for the trajectory segment for all of the axes
   * @param a maximal accelerations for the trajectory segment for all of the axes
   */
  bang_bang_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		  const std::vector<double> & coordinates,
		  const std::vector<double> & v,
		  const std::vector<double> & a);
  /**
   * Destructor.
   */
  ~bang_bang_trajectory_pose();

};

} // namespace trajectory_pose
} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_BANG_BANG_TRAJECTORY_POSE_H */
