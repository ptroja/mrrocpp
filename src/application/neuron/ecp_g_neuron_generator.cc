/*
 * ecp_g_neuron_generator.cpp
 *
 *  Created on: Jul 2, 2010
 *      Author: tbem
 */

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "ecp_g_neuron_generator.h"
#include "ecp_mp_neuron_sensor.h"
#include <ctime>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

neuron_generator::neuron_generator(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	reset();
}

neuron_generator::~neuron_generator()
{
	//delete neuron_sensor;
}

bool neuron_generator::first_step()
{
	sr_ecp_msg.message("neuron generator first step");
	printf("neuron generator first step\n");
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
	the_robot->ecp_command.instruction.motion_steps = 10;
	the_robot->ecp_command.instruction.value_in_step_no = 10 - 2;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;

	neuron_sensor = (ecp_mp::sensor::neuron_sensor*) sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR];
	neuron_sensor->startGettingTrajectory();
	return true;
}

bool neuron_generator::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	flushall();
	int i; // loop counter

	/*Check if entire trajectory was already sent, if so, finish the generator*/
	if (neuron_sensor->transmissionFinished()) {
		printf("End of transmission\n");
		return false;
	}

	if (neuron_sensor->current_period == 5) { //this section is not performed during breaking
		if (neuron_sensor->startBraking()) {
			breaking = true;

			printf("\n-------- breaking ----------\n");
			flushall();
		}

		flushall();
		// ------------ read the current robot position ----------
		actual_position_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		actual_position_matrix.get_xyz_angle_axis(angle_axis_vector);
		angle_axis_vector.to_table(actual_position);
		// ------------ read the current robot position (end) ---------

		flushall();
		desired_position[0] = neuron_sensor->getCoordinates().x;
		desired_position[1] = neuron_sensor->getCoordinates().y;
		desired_position[2] = neuron_sensor->getCoordinates().z;
		desired_position[3] = position[3] = actual_position[3];
		desired_position[4] = position[4] = actual_position[4];
		desired_position[5] = position[5] = actual_position[5];

		if (breaking) {
			printf("current: %f\t %f\t %f\t %f\t %f\t %f\n", actual_position[0], actual_position[1], actual_position[2], actual_position[3], actual_position[4], actual_position[5]);
			printf("desired: %f\t %f\t %f\t %f\t %f\t %f\n", desired_position[0], desired_position[1], desired_position[2], desired_position[3], desired_position[4], desired_position[5]);
		}
	}

	int node = 6 - neuron_sensor->current_period;

	if (breaking) {//set the current period to 4 to avoid entering the above "if" condition
		neuron_sensor->current_period = 4;
	}

	for (i = 0; i < 6; i++) { //for all of the axes...
		if (desired_position[i] == actual_position[i]) {//if no motion in the axis
			position[i] = actual_position[i]; //position remains the same
			printf("%f\t", position[i]);
			if (breaking) {
				reached[i] = true;//current position is the desired position so the goal is reached
			}
			continue;
		}

		if (!breaking) { //if breaking, the direction of the motion is not changed
			if (desired_position[i] - actual_position[i] > 0) {
				k[i] = 1;
			} else {
				k[i] = -1;
			}
		}

		s[i] = fabs(desired_position[i] - actual_position[i]);//distance between current and desired position (s[i] is not updated during breaking

		if (breaking) {
			double a;//acceleration while breaking

			if (s[i] == 0 || v[i] == 0) {//if no motion in the axis or the velocity from which the robot breaks (v[i] is the velocity read in the moment when the breaking was started) is equal to 0
				reached[i] = true;
			}

			if (s[i] < (0.5 * v[i] * v[i] / 2)) {//if the generator is not able to break before reaching the desired position, the breaking is made with the maximal acceleration
				a = a_max[i];
			} else {
				a = (v[i] * v[i]) / (2 * s[i]);//acceleration with which the robot should break in a single axis to reach the desired position
			}

			if ((breaking_node * 0.02 * v[i] / 2 - breaking_node * breaking_node * 0.02 * 0.02 * a / 2) <= 0) {
				reached[i] = true;
			}

			if (!reached[i]) {//calculating the next position while breaking  (0.02 is the macrostep time)
				position[i] = actual_position[i] + (k[i] * (breaking_node * 0.02 * v[i] - breaking_node * breaking_node
						* 0.02 * 0.02 * a / 2));
			}

		} else {
			position[i] = actual_position[i] + (k[i] * (s[i] / 5) * node); //normal motion (not breaking), distance between desired and current position is divided by 5, desired position is reached in 5 macrosteps and added to actual position
			v[i] = (s[i] / 5) / 0.02;//current velocity, last v[i] is the velocity just before breaking, it is not updated during breaking

		}
		printf("%f\t", position[i]);
	}

	if (breaking) {
		breaking_node++;//increment the breaking node counter
	}

	printf("\n");
	flushall();

	// --------- send new position to the robot (EDP) --------------
	position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position));
	position_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);//send new position to the robot
	// --------- send new position to the robot (EDP) (end) --------------

	if (neuron_sensor->current_period == 1) {
		neuron_sensor->sendCoordinates(position[0], position[1], position[2]);
	}

	for (i = 0; i < 6; i++) {
		if (reached[i] == false) {//return true if the generator did not reach the desired position in all of the axes
			return true;
		}
	}

	return false;//return false, desired positions in all of the axes reached

}

double * neuron_generator::get_position()
{
	return position;
}

void neuron_generator::reset()
{

	int i;

	breaking = false;

	for (i = 0; i < 6; i++) {
		v[i] = 0.0;
	}

	for (i = 0; i < 6; i++) {
		a_max[i] = 0.1;
	}

	for (i = 0; i < 6; i++) {
		s[i] = 0.0;
	}

	for (i = 0; i < 6; i++) {
		k[i] = 0.0;
	}

	for (i = 0; i < 6; i++) {
		reached[i] = false;
	}

	breaking_node = 1;
}

void neuron_generator::set_breaking(bool breaking)
{
	this->breaking = breaking;
}

}//generator
}//common
}//ecp
}//mrrocpp
