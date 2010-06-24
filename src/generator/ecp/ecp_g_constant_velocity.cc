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

constant_velocity::constant_velocity(common::task::task& _ecp_task, bool _is_synchronised, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
		multiple_position<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose,
		ecp::common::generator::trajectory_interpolator::constant_velocity_interpolator,
		ecp::common::generator::velocity_profile_calculator::constant_velocity_profile> (_ecp_task) {
	this->pose_spec = pose_spec;
	this->axes_num = axes_num;
	this->vpc = velocity_profile_calculator::constant_velocity_profile();
	this->inter = trajectory_interpolator::constant_velocity_interpolator();
}

constant_velocity::~constant_velocity() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity::first_step() {

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
	} else if (motion_Type == lib::ABSOLUTE) {
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
				for (int i=0; i<6; i++) {
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

	int i; //loop counter

	if (pose_vector.empty()) {
		return false;
	}

	get_position * get_pos = new get_position(ecp_t, true, pose_spec, axes_num); //generator used to get the actual position of the robot
	get_pos->Move();

	pose_vector_iterator = pose_vector.begin();
	if (motion_type == lib::ABSOLUTE) {
		pose_vector_iterator->start_position = get_pos->get_position_vector();//get actual position of the robot
	} else if (motion_type == lib::RELATIVE) {
		pose_vector_iterator->start_position = 0;
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
			return false;//calculate velocities for all of the axes according to the longest needed time
		}
		pose_vector_iterator++;
	}

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
	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;
	vector<double> joint_velocity(axes_num, 0.05);
	if (!pose_vector.empty() && pose_spec != lib::ECP_JOINT) { //check if previous positions were provided in joint representation
		return false;
	}

	pose_spec = lib::ECP_JOINT;
	pose = ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose(lib::ECP_JOINT, coordinates, joint_velocity); //create new trajectory pose
	for (int j = 0; j < axes_num; j++) { //set the v_max and calculate v_r velocities
		pose.v_max.push_back(1.5);
		pose.v_r[j] = pose.v[j] * pose.v_max[j];
	}

	if (!pose_vector.empty()) {//set the start position as the desired position of the previous pose
		pose_vector_iterator = pose_vector.end();
		pose.start_position = pose_vector_iterator->start_position;
	}

	pose_vector.push_back(pose); //put new trajectory pose into a pose vector

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
