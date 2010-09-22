/*
 * constant_velocity_interpolator.cc
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
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

/*bool constant_velocity_interpolator::interpolate_angle_axis_absolute_pose_transformed_into_relative(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

	vector<double> coordinates (it->axes_num);
	//vector<double> general_temp (it->axes_num);

	double start_position_array[6];
	double coordinate_backup[6];

	lib::Homog_matrix begining_frame;
	lib::Homog_matrix goal_frame;
	lib::Homog_matrix begining_frame_with_current_translation;

	lib::Xyz_Angle_Axis_vector step_of_total_increment_vector;
	lib::Xyz_Angle_Axis_vector tmp_angle_axis_vector;

	int z;

	for (z = 0; z < 6; z++) {
		start_position_array[z] = it->start_position[z];
	}

	begining_frame.set_from_xyz_angle_axis(start_position_array);
	goal_frame.set_from_xyz_angle_axis(start_position_array);

	for (int i = 0; i < it->interpolation_node_no; i++) {
			for (int j = 0; j < it->axes_num; j++) {
				coordinates[j] = it->k[j] * mc * it->v_r[j];
			}

		for (z = 0; z < 6; z++) {
			coordinate_backup[z] = coordinates[z];
		}

		begining_frame_with_current_translation = begining_frame;
		begining_frame_with_current_translation.set_translation_vector(goal_frame);

		step_of_total_increment_vector =
					lib::V_tr(!(lib::V_tr(!begining_frame_with_current_translation
							* goal_frame))) * lib::Xyz_Angle_Axis_vector(coordinate_backup);

		goal_frame = goal_frame * lib::Homog_matrix(step_of_total_increment_vector);

		goal_frame.get_xyz_angle_axis(tmp_angle_axis_vector);
		tmp_angle_axis_vector.to_vector(coordinates);

		//TODO add checking the correctness of the returned values

		cv.push_back(coordinates);
	}

	return true;
}*/

double constant_velocity_interpolator::generate_relative_coordinate(int node_counter, std::vector <ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int axis_num, const double mc) {
	return it->k[axis_num] * mc * it->v_r[axis_num];
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
