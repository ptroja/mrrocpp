/**
 * @file
 * @brief Contains declarations of the methods of spline_trajectory_pose class.
 * @author rtulwin
 * @ingroup generators
 */

#if !defined(_ECP_SPLINE_TRAJECTORY_POSE_H)
#define  _ECP_SPLINE_TRAJECTORY_POSE_H

#include "base/lib/trajectory_pose/trajectory_pose.h"
#include <vector>

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

/**
 * @brief Class is a container used by generators performing spline polynomial interpolation.
 *
 * One instance of the class contains information about one segment of the trajectory (single movement in one particular direction).
 *
 * @author rtulwin
 * @ingroup trajectory_pose
 */
class spline_trajectory_pose : public trajectory_pose
{
public:
    /**
     * Empty constructor.
     */
    spline_trajectory_pose(void);
    /**
     * Constructor which initiates some variables (those which can be found in the file containing trajectory).
     * @param arm_type representation used in the given pose
     * @param coordinates desired position for all of the axes
     * @param v maximal velocities for the trajectory segment for all of the axes
     */
    spline_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
                    const std::vector<double> & coordinates,
                    const std::vector<double> & v);
    /**
     * Destructor.
     */
    ~spline_trajectory_pose();
};

} // namespace trajectory_pose
} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif // _ECP_SPLINE_TRAJECTORY_POSE_H
