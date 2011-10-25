/**
 * @file
 * @brief Contains definitions of the methods of constant_velocity_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#include "constant_velocity_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

using namespace std;

constant_velocity_interpolator::constant_velocity_interpolator() {
	// TODO Auto-generated constructor stub

}

constant_velocity_interpolator::~constant_velocity_interpolator() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity_interpolator::interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

	vector<double> coordinates (it->axes_num);
	for (int i = 0; i < it->interpolation_node_no; i++) {
		for (int j = 0; j < it->axes_num; j++) {
			coordinates[j] = it->k[j] * mc * it->v_r[j];
		}
		cv.push_back(coordinates);
	}

	return true;
}

bool constant_velocity_interpolator::interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

	vector<double> coordinates (it->axes_num);
	for (int i = 0; i < it->interpolation_node_no; i++) {
		for (int j = 0; j < it->axes_num; j++) {
			coordinates[j] = it->start_position[j] + (it->k[j] * (i+1) * mc * it->v_r[j]);
		}
		cv.push_back(coordinates);
	}

	return true;
}

double constant_velocity_interpolator::generate_relative_coordinate(int node_counter, std::vector <ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int axis_num, const double mc) {
	return it->k[axis_num] * mc * it->v_r[axis_num];
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
