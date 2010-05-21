/**
 * \file bang_bang_trajectory_pose.h
 * \brief Header file for bang_bang_trajectory_pose
 *
 * Contains declaration of bang_bang_trajectory_pose class and its methods.
 */

#if !defined(_ECP_BANG_BANG_TRAJECTORY_POSE_H)
#define  _ECP_BANG_BANG_TRAJECTORY_POSE_H

#include "ecp_mp/trajectory_pose/trajectory_pose.h"
#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

/**
 * Class is a container used by smooth trajectory generator. One instance of the class contains information about one segment of the trajectory (single movement in one particular direction).
 */
class bang_bang_trajectory_pose : public trajectory_pose {
public:
  /**
   * Initial velocity for the pose, for each axis.
   */
  vector<double> v_p;
  /**
   * Final velocity for the pose, for each axis.
   */
  vector<double> v_k;
  /**
   * Maximal velocity for the movement, for each axis. Percent of the v_max value. Should be a value between 0 - 1.
   */
  vector<double> v;
  /**
   * Maximal velocity set for the given robot (v_r = v * v_max).
   */
  vector<double> v_max;
  /**
   * Maximal acceleration for the motion, for each axis.
   */
  vector<double> a;
  /**
   * Number of the macrostep in which the first part of the movement ends (first out of three).
   */
  vector<double> przysp;
  /**
   * Number of the macrostep in which the seconde part of the movement ends.
   */
  vector<double> jedn;
  /**
   * Distance covered in the second part of the movement.
   */
  vector<double> s_jedn;
  /**
   * Distance covered in the first part of the movement.
   */
  vector<double> s_przysp;
  /**
   * Initial position for the pose.
   */
  vector<double> start_position;
  /**
   * Maximal acceleration for the given segment (pose) (calculated, can be smaller or equal to a).
   */
  vector<double> a_r;
  /**
   * Maximal velocity for the given segment (pose) (calculated, can be smaller or equal to v).
   */
  vector<double> v_r;
  /**
   * Motion kinematic_model_with_tool.
   */
  vector<int> model;

  /**
   * Empty constructor.
   */
  bang_bang_trajectory_pose (void);
  /**
   * Constructor which initiates some variables (those which can be found in the file containing trajectory).
   * \param at representation used in the given pose
   * \param coordinates desired position for all of the axes
   * \param vv maximal velocities for the trajectory segment for all of the axes
   * \param aa maximal accelerations for the trajectory segment for all of the axes
   * \param axes_num number of axes
   */
  bang_bang_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		  const vector<double> & coordinates,
		  const vector<double> & v,
		  const vector<double> & a,
		  const int axes_num);
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
