/**
 * \file ecp_g_newsmooth.cc
 * \brief newsmooth class and its methods
 *
 * Contains bodies of the methods of newsmooth class.
 */

#include "generator/ecp/ecp_g_newsmooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

newsmooth::newsmooth(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
				multiple_position<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose,
				ecp::common::generator::trajectory_interpolator::bang_bang_interpolator,
				ecp::common::generator::velocity_profile_calculator::bang_bang_profile> (_ecp_task) {
	this->pose_spec = pose_spec;
	this->axes_num = axes_num;
	this->vpc = velocity_profile_calculator::bang_bang_profile();
	this->inter = trajectory_interpolator::bang_bang_interpolator();

	create_velocity_vectors(axes_num);
}

newsmooth::~newsmooth() {

}

bool newsmooth::calculate() {

	return true;
}

void newsmooth::print_pose_vector() {
	printf("------------------ Pose List ------------------\n");
	pose_vector_iterator = pose_vector.begin();
	int z;
	for (int k = 0; k < pose_vector.size(); k++) {
		printf("s:\t");
		for (z = 0; z < pose_vector_iterator->s.size(); z++) {
			printf("%f\t", pose_vector_iterator->s[z]);
		}
		printf("\n");
		printf("k:\t");
		for (z = 0; z < pose_vector_iterator->k.size(); z++) {
			printf("%f\t", pose_vector_iterator->k[z]);
		}
		printf("\n");
		printf("times:\t");
		for (z = 0; z < pose_vector_iterator->s.size(); z++) {
			printf("%f\t", pose_vector_iterator->times[z]);
		}
		printf("\n");
		printf("v_r:\t");
		for (z = 0; z < pose_vector_iterator->v_r.size(); z++) {
			printf("%f\t", pose_vector_iterator->v_r[z]);
		}
		printf("\n");
		printf("a_r:\t");
		for (z = 0; z < pose_vector_iterator->a_r.size(); z++) {
			printf("%f\t", pose_vector_iterator->a_r[z]);
		}
		printf("\n");
		printf("t: %f\n", pose_vector_iterator->t);
		flushall();
		pose_vector_iterator++;
	}
}

void newsmooth::print_coordinate_vector() {
	coordinate_vector_iterator = coordinate_vector.begin();
	printf("coordinate_vector_size: %d\n", coordinate_vector.size());
	for (int i = 0; i < coordinate_vector.size(); i++) {
		tempIter = (*coordinate_vector_iterator).begin();
		printf("%d:\t", (i+1));
		for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
			printf(" %f\t", *tempIter);
		}
		coordinate_vector_iterator++;
		printf("\n");
	}
	flushall();
}

void newsmooth::create_velocity_vectors(int axes_num) {
	joint_velocity = vector<double>(axes_num, 0.05);
	joint_max_velocity = vector<double>(axes_num, 1.5);
	joint_acceleration = vector<double>(axes_num, 0.05);
	joint_max_acceleration = vector<double>(axes_num, 7.0);
	motor_velocity = vector<double>(axes_num, 0.05);
	motor_max_velocity = vector<double>(axes_num, 200.0);
	motor_acceleration = vector<double>(axes_num, 0.05);
	motor_max_acceleration = vector<double>(axes_num, 150.0);
	euler_zyz_velocity= vector<double>(axes_num, 0.01);//TODO check if this is a reasonable value
	euler_zyz_max_velocity = vector<double>(axes_num, 5.0);
	euler_zyz_acceleration = vector<double>(axes_num, 0.01);//TODO check if this is a reasonable value
	euler_zyz_max_acceleration = vector<double>(axes_num, 5.0);
	angle_axis_velocity = vector<double>(axes_num, 0.01);//TODO check if this is a reasonable value
	angle_axis_max_velocity = vector<double>(axes_num, 5.0);
	angle_axis_acceleration = vector<double>(axes_num, 0.01);//TODO check if this is a reasonable value
	angle_axis_max_acceleration = vector<double>(axes_num, 5.0);
}

bool newsmooth::load_trajectory_pose(const vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const vector<double> & v, const vector<double> & v_max) {

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
