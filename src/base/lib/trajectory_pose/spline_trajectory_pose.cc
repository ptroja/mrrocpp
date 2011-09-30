/**
 * @file
 * @brief Contains definitions of the methods of spline_trajectory_pose class.
 * @author rtulwin
 * @ingroup generators
 */

#include "base/lib/trajectory_pose/spline_trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

using namespace std;

spline_trajectory_pose::spline_trajectory_pose(void)
{

}

spline_trajectory_pose::spline_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
                const vector<double> & coordinates,
                const vector<double> & v) :
                trajectory_pose (arm_type, coordinates) {

        /*this->v = v;

        v_max = vector<double>(axes_num);
        start_position = vector<double>(axes_num);
        v_r = vector<double>(axes_num);*/
}

spline_trajectory_pose::~spline_trajectory_pose() {

}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
