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
	nmc = 10;
	mc = nmc * STEP;

	create_velocity_vectors(axes_num);
}

constant_velocity::~constant_velocity() {

}

bool constant_velocity::first_step() {

	if (debug) {
		printf("\n##################################### first_step ####################################\n");
		flushall();
	}

	if (!calculated || !interpolated) {
		reset();
		return false;
	}

	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.motion_steps = nmc;
	the_robot->ecp_command.instruction.value_in_step_no = nmc - 2;
	the_robot->communicate_with_edp = false;

	if (motion_type == lib::RELATIVE) {
		the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	} else if (motion_type == lib::ABSOLUTE) {
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	} else {
		sr_ecp_msg.message("Wrong motion type");
		throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
	}

	switch (pose_spec) {
		case lib::ECP_XYZ_ANGLE_AXIS || lib::ECP_XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
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
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			break;
		case lib::ECP_JOINT:
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			break;
		default:
			reset();
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}

	coordinate_vector_iterator = coordinate_vector.begin();
	sr_ecp_msg.message("Moving...");
	return true;
}

bool constant_velocity::next_step() {
	if (debug) {
		//printf("\n##################################### next_step ####################################\n");
		flushall();
	}

	int i;//loop counter

	if (coordinate_vector.empty()) {

		sr_ecp_msg.message("No coordinates generated");
		reset();
		return false;
	}

	the_robot->communicate_with_edp = true;//turn on the communication with EDP
	the_robot->ecp_command.instruction.instruction_type = lib::SET;

	double coordinates[axes_num];

	switch (pose_spec) {

		case lib::ECP_JOINT:

			tempIter = (*coordinate_vector_iterator).begin();
			for (i = 0; i < axes_num; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= *tempIter;
				if (debug) {
					printf("%f\t", *tempIter);
				}
				tempIter++;

			}
			if (debug) {
				printf("\n");
				flushall();
			}
			break;

		case lib::ECP_MOTOR:

			tempIter = (*coordinate_vector_iterator).begin();
			for (i = 0; i < axes_num; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= *tempIter;
				if (debug) {
					printf("%f\t", *tempIter);
				}
				tempIter++;

			}
			if (debug) {
				printf("\n");
				flushall();
			}
			break;

		case lib::ECP_XYZ_EULER_ZYZ:

			i = 0;

			for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
				coordinates[i] = *tempIter;
				i++;
			}

			homog_matrix.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(coordinates));
			homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

			break;

		case lib::ECP_XYZ_ANGLE_AXIS:

			i = 0;

			for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
				coordinates[i] = *tempIter;
				i++;
			}

			homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(coordinates));
			homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

			break;

		default:
			reset();
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	coordinate_vector_iterator++;
	if (coordinate_vector_iterator == coordinate_vector.end()) {
		reset();//reset the generator, set generated and calculated flags to false, flush coordinate and pose lists
		sr_ecp_msg.message("Motion finished");
		return false;
	} else {
		return true;
	}
}

bool constant_velocity::calculate_interpolate() {

	if (debug) {
		printf("\n##################################### calculate_interpolate ####################################\n");
		flushall();
	}

	if (pose_vector.empty()) {
		sr_ecp_msg.message("No loaded poses");
		return false;
	}

	get_initial_position();

	calculated = calculate();

	//---------------- DEGUG --------------------
	if (debug) {
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
			printf("t: %f\n", pose_vector_iterator->t);
			flushall();
			pose_vector_iterator++;
		}
	}
	//------------------ DEBUG END ---------------

	interpolated = interpolate();

	//---------------- DEGUG --------------------
	if (debug) {
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
	//------------------ DEBUG END ---------------

	return calculated && interpolated;
}

void constant_velocity::get_initial_position() {
	pose_vector_iterator = pose_vector.begin();
	if (motion_type == lib::ABSOLUTE) {
		get_position * get_pos = new get_position(ecp_t, pose_spec, axes_num); //generator used to get the actual position of the robot
		get_pos->Move();

		//---------------- DEGUG --------------------

		if (debug) {
			printf("actual position vector, size: %d\n", get_pos->get_position_vector().size());
			for (int j = 0; j < get_pos->get_position_vector().size(); j++) {
				printf("%f\t", get_pos->get_position_vector()[j]);
			}
			printf("\n");
			flushall();
		}

		//------------------ DEBUG END ---------------

		pose_vector_iterator->start_position = get_pos->get_position_vector();//get actual position of the robot
		delete get_pos;
	} else if (motion_type == lib::RELATIVE) {
		pose_vector_iterator->start_position = vector<double>(axes_num,0);
	} else {
		sr_ecp_msg.message("Wrong motion type");
		throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
	}
}

bool constant_velocity::calculate() {

	sr_ecp_msg.message("Calculating...");
	int i;//loop counter

	for (i = 0; i < pose_vector.size(); i++) {//calculate distances, directions, times and velocities for each pose and axis

		if(motion_type == lib::ABSOLUTE) {//absolute type of motion
			if (!vpc.calculate_absolute_distance_direction_pose(pose_vector_iterator)) {
				return false;
			}
		} else if(motion_type == lib::RELATIVE) {//relative type of motion
			if (!vpc.calculate_relative_distance_direction_pose(pose_vector_iterator)) {
				return false;
			}
		} else {
			sr_ecp_msg.message("Wrong motion type");
			throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
		}

		if(!vpc.calculate_time_pose(pose_vector_iterator) ||//calculate times for each of the axes
		!vpc.calculate_pose_time(pose_vector_iterator, mc) ||//calculate the longest time from each of the axes and set it as the pose time (also extend the time to be the multiplcity of a single macrostep time)
		!vpc.calculate_constant_velocity_pose(pose_vector_iterator)) {//calculate velocities for all of the axes according to the longest needed time
			return false;
		}

		//calculate the number of the macrosteps for the pose
		pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / mc);

		if (debug) {
			printf("interpolation node no: %d\n", pose_vector_iterator->interpolation_node_no);
		}

		pose_vector_iterator++;
	}

	return true;
}

bool constant_velocity::interpolate() {

	sr_ecp_msg.message("Interpolating...");

	if (!calculated) {
		sr_ecp_msg.message("Cannot perform interpolation. Trajectory not calculated.");
		return false;
	}

	coordinate_vector.clear();
	coordinate_vector_iterator = coordinate_vector.begin();
	pose_vector_iterator = pose_vector.begin();

	int i; //loop counter

	bool trueFlag = true;//flag set to false if interpolation is not successful at some point

	if (motion_type == lib::ABSOLUTE) {
		for (i = 0; i < pose_vector.size(); i++) {//interpolate trajectory, fill in the coordinate list
			if(inter.interpolate_absolute_pose(pose_vector_iterator, coordinate_vector, mc) == false) {
				trueFlag = false;
			}
			pose_vector_iterator++;
		}
	} else if (motion_type == lib::RELATIVE) {
		for (i = 0; i < pose_vector.size(); i++) {//interpolate trajectory, fill in the coordinate list
			if(inter.interpolate_relative_pose(pose_vector_iterator, coordinate_vector, mc) == false) {
				trueFlag = false;
			}
			pose_vector_iterator++;
		}
	} else {
		sr_ecp_msg.message("Wrong motion type");
		throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
	}

	return trueFlag;
}

void constant_velocity::set_axes_num(int axes_num) {
	this->axes_num = axes_num;
	create_velocity_vectors(axes_num);
}

void constant_velocity::create_velocity_vectors(int axes_num) {
	joint_velocity = vector<double>(axes_num, 0.05);
	joint_max_velocity = vector<double>(axes_num, 1.5);
	motor_velocity = vector<double>(axes_num, 0.05);
	motor_max_velocity = vector<double>(axes_num, 200.0);
	euler_zyz_velocity= vector<double>(axes_num, 0.05);//TODO check if this is a reasonable value
	euler_zyz_max_velocity = vector<double>(axes_num, 5.0);
	angle_axis_velocity = vector<double>(axes_num, 0.05);//TODO check if this is a reasonable value
	angle_axis_velocity = vector<double>(axes_num, 5.0);
}

//--------------- METHODS USED TO LOAD POSES ----------------

bool constant_velocity::load_absolute_joint_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_JOINT, joint_velocity, joint_max_velocity);
}

bool constant_velocity::load_relative_joint_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_JOINT, joint_velocity, joint_max_velocity);
}

bool constant_velocity::load_absolute_motor_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_MOTOR, motor_velocity, motor_max_velocity);
}

bool constant_velocity::load_relative_motor_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_MOTOR, motor_velocity, motor_max_velocity);
}

bool constant_velocity::load_absolute_euler_zyz_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_max_velocity);
}

bool constant_velocity::load_relative_euler_zyz_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_max_velocity);
}

bool constant_velocity::load_absolute_angle_axis_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_max_velocity);
}

bool constant_velocity::load_relative_angle_axis_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_max_velocity);
}

bool constant_velocity::load_trajectory_pose(const vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const vector<double> & v, const vector<double> & v_max) {

	if (!pose_vector.empty() && this->pose_spec != pose_spec) { //check if previous positions were provided in joint representation

		sr_ecp_msg.message("Representation different than the previous one");
		return false;
	}

	if (!pose_vector.empty() && this->motion_type != motion_type) {

		sr_ecp_msg.message("Wrong motion type");
		return false;
	}

	this->motion_type = motion_type;
	this->pose_spec = pose_spec;

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose; //new trajectory pose
	pose = ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose(pose_spec, coordinates, v); //create new trajectory pose
	pose.v_max = v_max; //set the v_max vector

	for (int j = 0; j < axes_num; j++) { //calculate v_r velocities
		pose.v_r[j] = pose.v[j] * pose.v_max[j];
	}

	if (motion_type == lib::ABSOLUTE) {
		if (!pose_vector.empty()) {//set the start position of the added pose as the desired position of the previous pose
			pose.start_position = pose_vector.back().coordinates;
		}
	}

	pose_vector.push_back(pose); //put new trajectory pose into a pose vector

	return true;
}
//--------------- METHODS USED TO LOAD POSES END ----------------

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
