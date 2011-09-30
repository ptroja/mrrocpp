/**
 * @file
 * @brief Contains definitions of the methods of spline_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#include "spline_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

using namespace std;

spline_interpolator::spline_interpolator()
{
    // TODO Auto-generated constructor stub
}

spline_interpolator::~spline_interpolator()
{
        // TODO Auto-generated destructor stub
}

bool spline_interpolator::interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {



        return true;
}

bool spline_interpolator::interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {



        return true;
}

double spline_interpolator::generate_relative_coordinate(int node_counter, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, int axis_num, double mc)
{

        return 0.0;
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
