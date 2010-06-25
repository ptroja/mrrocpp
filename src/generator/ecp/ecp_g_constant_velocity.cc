/*
 * ecp_g_constant_velocity.cc
 *
 *  Created on: May 18, 2010
 *      Author: rtulwin
 */

#include "ecp_g_constant_velocity.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

constant_velocity::constant_velocity(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
		multiple_position<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose,
		ecp::common::generator::trajectory_interpolator::constant_velocity_interpolator,
		ecp::common::generator::velocity_profile_calculator::constant_velocity_profile> (_ecp_task) {
	this->pose_spec = pose_spec;
	this->axes_num = axes_num;
	this->vpc = velocity_profile_calculator::constant_velocity_profile();
	this->inter = trajectory_interpolator::constant_velocity_interpolator();
	motion_type = lib::ABSOLUTE;
}

constant_velocity::~constant_velocity() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity::first_step() {
	printf("##################################### first_step ####################################\n");
	flushall();
	if (!calculated || !interpolated) {
		return false;
	}

	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
	the_robot->ecp_command.instruction.motion_steps = 10;
	the_robot->ecp_command.instruction.value_in_step_no = 8;
	the_robot->ecp_command.instruction.instruction_type = lib::SET;

	if (motion_type == lib::RELATIVE) {
		the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	} else if (motion_type == lib::ABSOLUTE) {
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	} else {
		//TODO throw exception
	}

	switch (pose_spec) {
		case lib::ECP_XYZ_ANGLE_AXIS || lib::ECP_XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
			the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
			if (motion_type == lib::RELATIVE) {
				the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
				for (int i=0; i<axes_num; i++) {
					the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
				}
			} else {
				the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			}
			break;
		case lib::ECP_MOTOR:
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			break;
		case lib::ECP_JOINT:
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			break;
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}

	coordinate_vector_iterator = coordinate_vector.begin();

	return true;
}

bool constant_velocity::next_step() {
	printf("##################################### next_step ####################################\n");
	flushall();
	int i;//loop counter

	if (coordinate_vector.empty()) {
		return false;
	}

	switch (pose_spec)
		{

		case lib::ECP_XYZ_EULER_ZYZ:

			//homog_matrix.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(*coordinate_vector_iterator));
			//homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

			break;

		case lib::ECP_XYZ_ANGLE_AXIS:

			//homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(*coordinate_vector_iterator));
			//homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

			break;

		case lib::ECP_JOINT:

			for (i = 0; i < axes_num; i++) {
				//the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
					//	= coordinate_list_iterator->coordinate[i];
			}

			break;

		case lib::ECP_MOTOR:

			for (i = 0; i < axes_num; i++) {
				//the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
					//	= coordinate_list_iterator->coordinate[i];
			}

			break;

		default:
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	if (coordinate_vector_iterator == coordinate_vector.end()) {
		return false;
	} else {
		return true;
	}
}

bool constant_velocity::calculate_interpolate() {

	printf("##################################### calculate_interpolate ####################################\n");
	flushall();
	int i; //loop counter

	if (pose_vector.empty()) {
		return false;
	}

	pose_vector_iterator = pose_vector.begin();
	if (motion_type == lib::ABSOLUTE) {
		get_position * get_pos = new get_position(ecp_t, pose_spec, axes_num); //generator used to get the actual position of the robot
		get_pos->Move();

		printf("wektor pozycji o rozmiarze: %d\n", get_pos->get_position_vector().size());
		for (int j = 0; j < get_pos->get_position_vector().size(); j++) {
			printf("%f\t", get_pos->get_position_vector()[j]);
		}
		printf("\n");
		flushall();

		pose_vector_iterator->start_position = get_pos->get_position_vector();//get actual position of the robot
		delete get_pos;
	} else if (motion_type == lib::RELATIVE) {
		pose_vector_iterator->start_position = vector<double>(axes_num,0);
	} else {
		//TODO throw exception
	}

	for (i = 0; i < pose_vector.size(); i++) {//calculate distances and directions for each pose and axis

		if(motion_type == lib::ABSOLUTE) {
			if (!vpc.calculate_absolute_distance_direction_pose(pose_vector_iterator)) {
				return false;
			}
		} else if(motion_type == lib::RELATIVE) {
			if (!vpc.calculate_relative_distance_direction_pose(pose_vector_iterator)) {
				return false;
			}
		} else {
			//TODO throw exception
		}

		if(!vpc.calculate_time_pose(pose_vector_iterator) ||//calculate times for each of the axes
		!vpc.calculate_pose_time(pose_vector_iterator) ||//calculate the longest time from each of the axes and set it as the pose time
		!vpc.calculate_constant_velocity_pose(pose_vector_iterator)) {
			printf("nieudane inne calculate\n");
			return false;//calculate velocities for all of the axes according to the longest needed time
		}
		pose_vector_iterator++;
	}

	//---------------- DEGUG --------------------

	printf("------------------ Pose List ------------------\n");
	pose_vector_iterator = pose_vector.begin();
	int z;
	for (int k = 0; k < pose_vector.size(); k++) {
		printf("s:\t");
		for (z = 0; z < pose_vector_iterator->s.size(); z++) {
			printf("%f\t", pose_vector_iterator->s[z]);
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
		flushall();
		pose_vector_iterator++;
	}
	//------------------ DEBUG END ---------------

	calculated = true;

	coordinate_vector.clear();
	coordinate_vector_iterator = coordinate_vector.begin();
	pose_vector_iterator = pose_vector.begin();

	if (motion_type == lib::ABSOLUTE) {
		interpolated = inter.interpolate_absolute(pose_vector_iterator, coordinate_vector_iterator); //interpolate trajectory, fill in the coordinate list
	} else if (motion_type == lib::RELATIVE) {
		interpolated = inter.interpolate_relative(pose_vector_iterator, coordinate_vector_iterator); //interpolate trajectory, fill in the coordinate list
	} else {
		//TODO throw exception
	}

	if (calculated && interpolated)
		return true;
	else {
		return false;
	}
}

bool constant_velocity::load_absolute_joint_trajectory_pose(vector<double> & coordinates) {
	printf("##################################### load absolute joint trajectory pose ####################################\n");
	flushall();
	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;
	vector<double> joint_velocity(axes_num, 0.05);

	//printf("debug 0\n");
	flushall();

	if (!pose_vector.empty() && pose_spec != lib::ECP_JOINT) { //check if previous positions were provided in joint representation
//		printf("pose vector not empty and pose spec != joint\n");
		flushall();
		return false;
	}

	//printf("debug 1\n");
	flushall();

	pose_spec = lib::ECP_JOINT;
	pose = ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose(lib::ECP_JOINT, coordinates, joint_velocity); //create new trajectory pose
	for (int j = 0; j < axes_num; j++) { //set the v_max and calculate v_r velocities
		pose.v_max[j] = 1.5;
		pose.v_r[j] = pose.v[j] * pose.v_max[j];
	}

	//printf("debug 2\n");
	flushall();

	if (!pose_vector.empty()) {//set the start position of the added pose as the desired position of the previous pose
		printf("ustawiam start position na poprzednie coordinates\n");
		printf("wektor coordinates o rozmiarze: %d\n", pose_vector.back().coordinates.size());
		for (int j = 0; j < pose_vector.back().coordinates.size(); j++) {
			printf("%f\t", pose_vector.back().coordinates[j]);
		}
		printf("\n");
		pose.start_position = pose_vector.back().coordinates;
	}

	//printf("debug 3\n");
	flushall();

	pose_vector.push_back(pose); //put new trajectory pose into a pose vector

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
