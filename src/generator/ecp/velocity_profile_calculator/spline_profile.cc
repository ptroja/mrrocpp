/**
 * @file
 * @brief Contains definitions of the methods of spline_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#include "spline_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

using namespace std;

spline_profile::spline_profile()
{

}

spline_profile::~spline_profile()
{

}

bool spline_profile::calculate_time(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, int i)
{

    return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
