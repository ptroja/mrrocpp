/**
 * @file ecp_g_neuron_generator.cpp
 * @brief Header file for neuron_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @author Rafal Tulwin (rtulwin@stud.elka.pw.edu.pl)
 * @ingroup neuron
 * @date 02.07.2010
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

/*================================Constructor=============================*//**
 * @brief Constructor along with task configurator.
 * @param _ecp_taks Reference to task configurator.
 */
neuron_generator::neuron_generator(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	reset();
}

/*==============================Destructor================================*//**
 * @brief Destructor.
 */
neuron_generator::~neuron_generator()
{
}

/*===============================first_step===============================*//**
 * @brief First step of neuron generator
 * @details Initializes instruction for robot and sends information to VSP that
 * new trajectory execution is starting, therefore requests first coordinates
 * of a trajectory.
 */
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

	//get neuron sensor and send information about starting new trajectory.
	neuron_sensor = (ecp_mp::sensor::neuron_sensor*) sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR];
	neuron_sensor->startGettingTrajectory();
	return true;
}

/*================================next_step===============================*//**
 * @brief next step for neuron generator.
 * @details Next step interpolates 5 consecutive macro steps basing on received
 * coordinates from VSP. It also received information whether to start breaking
 * or not.
 */
bool neuron_generator::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	flushall();

	//Check if stop button in VSP was pressed
	if (neuron_sensor->stop()) {
		printf("VSP Stop button pressed\n");
		return false;
	}

	//when current period == 5 then new data from VSP are ready, therefore
	//generator has to interpolate this coordinates for 5 macro steps.
	//this section is not performed in breaking phase, during which in every
	//next step period is set to 4
	if (neuron_sensor->current_period == 5) {
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

		//get desired position for robot.
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

	if (breaking) {
		//set the current period to 4 to avoid entering the above "if" condition.
		neuron_sensor->current_period = 4;
	}

	//for all of the axes...
	for (int i = 0; i < 6; i++) {
		//if no motion in the axis
		if (desired_position[i] == actual_position[i]) {

			//position remains the same
			position[i] = actual_position[i];

			if (breaking) {
				//current position is the desired position so the goal is reached
				reached[i] = true;
			}
			continue;
		}

		//if breaking, the direction of the motion is not changed
		if (!breaking) {
			if (desired_position[i] - actual_position[i] > 0) {
				k[i] = 1;
			} else {
				k[i] = -1;
			}
		}

		//distance between current and desired position (s[i] is not updated during breaking
		s[i] = fabs(desired_position[i] - actual_position[i]);

		if (breaking) {
			//acceleration while breaking
			double a;

			//if no motion in the axis or the velocity from which the robot
			//breaks (v[i] is the velocity read in the moment when the breaking
			//was started) is equal to 0
			if (s[i] == 0 || v[i] == 0) {
				reached[i] = true;
			}

			//if the generator is not able to break before reaching the desired
			//position, the breaking is made with the maximal acceleration
			//otherwise calculate an acceleration with which a robot should
			//break in a single axis to raech the desired position.
			if (s[i] < (0.5 * v[i] * v[i] / 2)) {
				a = a_max[i];
			} else {
				a = (v[i] * v[i]) / (2 * s[i]);
			}

			if ((breaking_node * 0.02 * v[i] / 2 - breaking_node * breaking_node * 0.02 * 0.02 * a / 2) <= 0) {
				reached[i] = true;
			}

			//calculating the next position while breaking (0.02 is the macrostep time)
			if (!reached[i]) {
				position[i] = actual_position[i] + (k[i] * (breaking_node * 0.02 * v[i] - breaking_node * breaking_node
						* 0.02 * 0.02 * a / 2));
			}

		} else {
			//normal motion (not breaking), distance between desired and
			//current position is divided by 5, desired position is reached in
			//5 macrosteps and added to actual position
			position[i] = actual_position[i] + (k[i] * (s[i] / 5) * node);

			//current velocity, last v[i] is the velocity just before breaking,
			//it is not updated during breaking
			v[i] = (s[i] / 5) / 0.02;

		}
	}

	//increment the breaking node counter if in breaking phase
	if (breaking) {
		breaking_node++;
	}

	flushall();

	// --------- send new position to the robot (EDP) --------------
	position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position));
	//send new position to the robot
	position_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	// --------- send new position to the robot (EDP) (end) --------------

	if (neuron_sensor->current_period == 1) {
		neuron_sensor->sendCoordinates(position[0], position[1], position[2]);
	}

	//return true if the generator did not reach the desired position in all of
	//the axes
	for (int i = 0; i < 6; i++) {
		if (reached[i] == false) {
			return true;
		}
	}

	//return false, desired positions in all of the axes reached
	return false;
}

/**
 * @brief Returns current robot position.
 * @return Current robot position.
 */
double * neuron_generator::get_position()
{
	return position;
}

/**
 * @brief Resets generator between consecutive call of Move() method.
 * @detail Resets all of the temporary variables. It is necessary to call the
 * reset between the calls of the generator Move() method.
 */
void neuron_generator::reset()
{
	for (int i = 0; i < 6; i++) {
		v[i] = 0.0;
		a_max[i] = 0.1;
		s[i] = 0.0;
		k[i] = 0.0;
		reached[i] = false;
	}

	breaking = false;
	breaking_node = 1;
}

}//generator
}//common
}//ecp
}//mrrocpp
