/**
 * @file trajectory_pose.h
 * @rief Header file for trajectory_pose
 *
 * Contains declaration of trajectory_pose class and its methods.
 */

#if !defined(_TRAJECTORY_POSE_H)
#define  _TRAJECTORY_POSE_H

#include "lib/com_buf.h"
#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

/**
 * Base class for all trajectory pose types. Contains fields common for  all trajectory pose descriptions.
 */
class trajectory_pose {
public:
  /**
   * Representation used in the trajectory segment.
   */
  lib::ECP_POSE_SPECIFICATION arm_type;
  /**
   * Desired position
   */
  vector<double> coordinates;
  /**
   * Number of macrosteps in pose.
   */
  int interpolation_node_no;
  /**
   * Time needed to perform a movement (usually the biggest time stored in the vector times).
   */
  double t;
  /**
   * Times needed to perform a movement in each of the axes.
   */
  vector<double> times;
  /**
   * Direction of the motion. Either equal to 1 or -1.
   */
  vector<double> k;
  /**
   * Number of the given position in whole trajectory chain.
   */
  int pos_num;
  /**
   * Number of axes in which we want the robot to move in the given representation
   */
  int axes_num;
  /**
   * Vector of distances to be covered in all axes.
   */
  vector<double> s;
  /**
   * Empty constructor.
   */
  trajectory_pose(void);
  /**
     * Constructor which initiates some variables (those which can be found in the file containing trajectory).
     * @param arm_type representation used in the given pose
     * @param coordinates desired position for all of the axes
     */
  trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
			const vector<double> & coordinates);
  /**
   * Destructor.
   */
  ~trajectory_pose();

};

} // namespace trajectory_pose
} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_BANG_BANG_TRAJECTORY_POSE_H */
