/**
 * @file ecp_g_neuron_generator.cpp
 * @brief Header file for neuron_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @author Rafal Tulwin (rtulwin@gmail.com)
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

	//when current period == 5 then new data from VSP is ready, therefore
	//generator has to interpolate this coordinates for 5 macro steps.
	//this section is not performed in breaking phase, during which in every
	//next step period is set to 4
	if (neuron_sensor->current_period == 5) {
		//printf("period 5\n");
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

		last_but_one[0] = neuron_sensor->getLastButOne().x;
		last_but_one[1] = neuron_sensor->getLastButOne().y;
		last_but_one[2] = neuron_sensor->getLastButOne().z;



		if (breaking) {
			for (int i = 0; i < 3; i++) {
				normalized_vector[i] = (last_but_one[i] - desired_position[i]) /
						sqrt((last_but_one[0] - desired_position[0]) * (last_but_one[0] - desired_position[0]) +
							 (last_but_one[1] - desired_position[1]) * (last_but_one[1] - desired_position[1]) +
						     (last_but_one[2] - desired_position[2]) * (last_but_one[2] - desired_position[2]));
			}
			overshoot = (position[0] - desired_position[0]) * normalized_vector[0] + (position[1] - desired_position[1]) * normalized_vector[1] + (position[2] - desired_position[2]) * normalized_vector[2];
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

		if (!breaking) {
			if (desired_position[i] - actual_position[i] > 0) {
				k[i] = 1;
			} else {
				k[i] = -1;
			}
		} else {
			/*if (desired_position[i] - position[i] > 0) {
				k[i] = 1;
			} else {
				k[i] = -1;
			}*/
		}

		if (!breaking) {
			u[i] = fabs(desired_position[i] - actual_position[i]);
		} else {
			u[i] = desired_position[i] - position[i];
		}

		if (breaking) {

			if (k[i] == u[i]/fabs(u[i])) {
				change[i] = false;
			} else {
				change[i] = true;
				if (v[i] == 0) {
					if (u[i] != 0) {
						k[i] = u[i]/fabs(u[i]);//first direction setting
					}
				}
			}

			if(fabs(desired_position[i] - position[i]) < 0.0018 && v[i] < 0.005) {
				reached[i] = true;
				continue;
			}

			if (change[i] == false && (v[i]*v[i]) / (2 * a_max[i]) <= (fabs(u[i]) - (v[i] * 0.02))) {
				breaking_possible[i] = true;
			}

			if (change[i] == false && (v[i]*v[i]) / (2 * a_max[i]) >= (fabs(u[i]) - (v[i] * 0.02)) && breaking_possible[i]) {
				almost_reached[i] = true;
			}

			if (v[i] == 0 || ((v[i] > 0 && v[i] < v_max[i] && change[i] == false && reached[i] == false) && !almost_reached[i])) {//acceleration
				/*if (v[i] == 0 && change[i] == true) {
					change[i] = false;
					k[i] = -k[i];
				}*/
				//if ((fabs(u[i]) < 20) && (v[i] <= 0)) {
				//	continue;
				//
				s[i] = (a_max[i] * t * t)/2 + (v[i] * t);
				v[i] += a_max[i] * t;
				if (v[i] > v_max[i]) {
					v[i] = v_max[i];
				}
			} else if(v[i] > 0 && ((change[i] == true || reached[i] == true || (v[i]-v_max[i]) > 0.0001) || almost_reached[i])) {//breaking
				if (v[i] > v_max[i] && (v[i]-v_max[i])/t < a_max[i] && change[i] == false && reached[i] == false) {
					v[i] = v_max[i];
					s[i] = (((v[i]-v_max[i])/t) * t * t)/2 + (v[i] * t);
				} else {
					v[i] -= a_max[i] * t;
					s[i] = (a_max[i] * t * t)/2 + (v[i] * t);
				}
				if (v[i] < 0) {
					v[i] = 0;
					s[i] = 0;
				}

				if (change[i] == true && v[i] == 0) {
					change[i] = false;
					k[i] = -k[i];
				}
			} else { //uniform motion
				s[i] = v[i] * t;
			}
			position[i] += k[i] * s[i];
			printf("%f\t", (k[i] * s[i]));

		} else {
			//normal motion (not breaking), distance between desired and
			//current position is divided by 5, desired position is reached in
			//5 macrosteps and added to actual position
			position[i] = actual_position[i] + (k[i] * (u[i] / 5) * node);
			//current velocity, last v[i] is the velocity just before breaking,
			//it is not updated during breaking
			v[i] = (u[i] / 5) / 0.02;
			//printf("v[%d]: %f\n", i, v[i]);
		}
	}
	printf("\n");
	/*printf("k:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", k[0], k[1], k[2], k[3], k[4], k[5]);
	printf("s:\t %f\t %f\t %f\t %f\t %f\t %f\n", s[0], s[1], s[2], s[3], s[4], s[5]);
	printf("u:\t %f\t %f\t %f\t %f\t %f\t %f\n", u[0], u[1], u[2], u[3], u[4], u[5]);
	printf("v:\t %f\t %f\t %f\t %f\t %f\t %f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
	printf("v_max:\t %f\t %f\t %f\t %f\t %f\t %f\n", v_max[0], v_max[1], v_max[2], v_max[3], v_max[4], v_max[5]);
	printf("a_max:\t %f\t %f\t %f\t %f\t %f\t %f\n", a_max[0], a_max[1], a_max[2], a_max[3], a_max[4], a_max[5]);
	printf("chan:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", change[0], change[1], change[2], change[3], change[4], change[5]);
	printf("rea:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", reached[0], reached[1], reached[2], reached[3], reached[4], reached[5]);
	printf("a_rea:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", almost_reached[0], almost_reached[1], almost_reached[2], almost_reached[3], almost_reached[4], almost_reached[5]);
	printf("b_p:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", breaking_possible[0], breaking_possible[1], breaking_possible[2], breaking_possible[3], breaking_possible[4], breaking_possible[5]);*/


	//increment the breaking node counter if in breaking phase
	if (breaking) {
		breaking_node++;
	}

	flushall();

	if ((position[0] - desired_position[0]) * normalized_vector[0] + (position[1] - desired_position[1]) * normalized_vector[1] + (position[2] - desired_position[2]) * normalized_vector[2] < overshoot && breaking) {
		overshoot = (position[0] - desired_position[0]) * normalized_vector[0] + (position[1] - desired_position[1]) * normalized_vector[1] + (position[2] - desired_position[2]) * normalized_vector[2];
	}

	//printf("overshoot: %f\n", overshoot);

	//printf("pos:\t %f\t %f\t %f\t %f\t %f\t %f\n", position[0], position[1], position[2], position[3], position[4], position[5]);
	// --------- send new position to the robot (EDP) ---------------
	position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position));
	//send new position to the robot
	position_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	// --------- send new position to the robot (EDP) (end) --------------

	if (neuron_sensor->current_period == 1) {
		printf("coordiantes sent from current period = 1\n");
		neuron_sensor->sendCurrentPosition(position[0], position[1], position[2]);
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
 * @brief Returns time necessary to reach the desired position while breaking.
 * @return time necessary to reach the desired position while breaking (in seconds)
 */
double neuron_generator::get_breaking_time()
{
	return breaking_node * 0.02;
}

/**
 * @brief
 * @return the biggest value of the overshoot while breaking
 */
double neuron_generator::get_overshoot()
{
	return overshoot;
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
		u[i] = 0.0;
		a_max[i] = 0.15;
		v_max[i] = 0.15;
		s[i] = 0.0;
		k[i] = 0.0;
		reached[i] = false;
		change[i] = false;
		t = 0.02;
		almost_reached[i] = false;
		breaking_possible[i] = false;
		overshoot = 0;
	}

	breaking = false;
	breaking_node = 1;
}

}//generator
}//common
}//ecp
}//mrrocpp
