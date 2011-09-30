/**
 * @file
 * @brief Contains definitions of the methods of spline class.
 * @author rtulwin
 * @ingroup generators
 */

#include "ecp_g_spline.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;

spline::spline(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
        multiple_position<ecp_mp::common::trajectory_pose::spline_trajectory_pose,
        ecp::common::generator::trajectory_interpolator::spline_interpolator,
        ecp::common::generator::velocity_profile_calculator::spline_profile> (_ecp_task) {
}

spline::~spline() {

}

void spline::print_pose_vector() {

}

bool spline::calculate() {

    return true;
}

void spline::create_velocity_vectors(int axes_num) {

}

bool spline::load_absolute_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose) {

        return true;
}

bool spline::load_relative_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose) {

        return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
