/*
 * ecp_g_neuron_generator.cpp
 *
 *  Created on: Jul 2, 2010
 *      Author: tbem
 */

#include "ecp_g_neuron_generator.h"
#include "ecp_mp_neuron_sensor.h"
#include <time.h>

namespace mrrocpp{
namespace ecp{
namespace common{
namespace generator{

#define START_BREAKING			7

neuron_generator::neuron_generator(common::task::task& _ecp_task):generator(_ecp_task) {

	reset();
}

neuron_generator::~neuron_generator() {
	delete neuron_sensor;
}

bool neuron_generator::first_step(){
	ecp_t.sr_ecp_msg->message("neuron generator first step");
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
	//the_robot->communicate_with_edp = false;

	neuron_sensor=(ecp_mp::sensor::neuron_sensor*)sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR];
	neuron_sensor->startGettingTrajectory();
	neuron_sensor->counter=0;
	printf("firstStep: currentPeriod: %d\n",neuron_sensor->current_period);
	return true;
}

bool neuron_generator::next_step(){
	//the_robot->communicate_with_edp = true;
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	flushall();
	int i; //lop counter

	/*Check if entire trajectory was already sent, if so, finish the generator*/
	if(neuron_sensor->transmissionFinished()){
		printf("End of transmission %d\n",neuron_sensor->counter);
		flushall();
		return false;
	}

	//double general_temp[6];
	//double coordinate_backup[6];

	//lib::Homog_matrix goal_frame;
	//lib::Homog_matrix begining_frame;

	/*when current_period==0 get_reading is called, so new data is available*/
	//printf("current_period: %d\n", neuron_sensor->current_period);
	//printf("nextStep: currentPeriod: %d\n",neuron_sensor->current_period);

	if(neuron_sensor->current_period==5){
		if(neuron_sensor->getCommand()==START_BREAKING) {
			breaking=true;
			printf("\n-------- breaking ----------\n");
			flushall();
		}

		//printf("period 5: x:%lf y:%lf z:%lf\n",neuron_sensor->getCoordinates().x,neuron_sensor->getCoordinates().y,neuron_sensor->getCoordinates().z);
		flushall();
		// ------------ read the current robot position ----------
		if (!breaking) {
			actual_position_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
			actual_position_matrix.get_xyz_angle_axis(angle_axis_vector);
			angle_axis_vector.to_table(actual_position);
			}
		// ------------ read the current robot position (end) ---------

		//printf("actual_pos: \t");
		for (i = 0; i < 6; i++) {
			//printf("%f\t", actual_position[i]);
		}
		//printf("\n");
		flushall();
		desired_position[0] = neuron_sensor->getCoordinates().x;
		desired_position[1] = neuron_sensor->getCoordinates().y;
		desired_position[2] = neuron_sensor->getCoordinates().z;
		desired_position[3] = position[3] = actual_position[3];
		desired_position[4] = position[4] = actual_position[4];
		desired_position[5] = position[5] = actual_position[5];

		/*actual_position_matrix.set_from_xyz_angle_axis(actual_position);
		desired_position_matrix.set_from_xyz_angle_axis(desired_position);

		((!actual_position_matrix) * desired_position_matrix).get_xyz_angle_axis(angle_axis_vector);
		angle_axis_vector.to_table(desired_position);

		begining_frame.set_from_xyz_angle_axis(actual_position);
		goal_frame.set_from_xyz_angle_axis(actual_position);

		printf("x: %f\t y: %f\t z: %f\n %f\t %f\t %f\n", desired_position[0], desired_position[1], desired_position[2], desired_position[3], desired_position[4], desired_position[5]);
		flushall();*/
		if (breaking) {
			printf("desired: %f\t %f\t %f\t %f\t %f\t %f\n", desired_position[0], desired_position[1], desired_position[2], desired_position[3], desired_position[4], desired_position[5]);
		}
	}
/*
	printf("poza if\n");
	flushall();

	//interpolacja

	for (i = 0; i < 6; i++) {
		position[i] = 0;
	}

	for (i = 0; i < 6; i++) {
		coordinate_backup[i] = position[i];
	}

	lib::Homog_matrix begining_frame_with_current_translation = begining_frame;
	begining_frame_with_current_translation.set_translation_vector(goal_frame);

	lib::Xyz_Angle_Axis_vector step_of_total_increment_vector =
				lib::V_tr(!(lib::V_tr(!begining_frame_with_current_translation
						* goal_frame))) * lib::Xyz_Angle_Axis_vector(coordinate_backup);

	goal_frame = goal_frame * lib::Homog_matrix(step_of_total_increment_vector);

	lib::Xyz_Angle_Axis_vector tmp_angle_axis_vector;
	goal_frame.get_xyz_angle_axis(tmp_angle_axis_vector);
	tmp_angle_axis_vector.to_table(general_temp);

	for(int g = 0; g < 6; g++) {
		printf("%f\t", general_temp[g]);
	}
	printf("\n");


	position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(general_temp));
	position_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
*/

	int node = 6 - neuron_sensor->current_period;

	if(breaking){
		neuron_sensor->current_period=4;
	}

	//printf(" position: \t");
	for (i = 0; i < 6; i ++) {

		if (desired_position[i] == actual_position[i]) {//if no motion in the axis
			position[i] = actual_position[i];
			printf("%f\t", position[i]);
			if (breaking) {
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
		}

		s[i] = fabs(desired_position[i] - actual_position[i]);

		if (breaking) {
			//printf("v: %f ", v[i]);
			double a;

			if (s[i] == 0) {
				reached[i] = true;
			}

			if (s[i] < (0.5 * v[i] * v[i] / 2)) {
				printf("max: \t");
				a = a_max[i];
			} else {

				a = (v[i] * v[i]) / (2 * s[i]);
			}

			//printf("k: %f\t act pos + %f\t", k[i], (breaking_node * 0.02 * v[i]/2 - breaking_node * breaking_node * 0.02 * 0.02 * a / 2));
			if ((breaking_node * 0.02 * v[i]/2 - breaking_node * breaking_node * 0.02 * 0.02 * a / 2) <= 0) {
				reached[i] = true;
			}

			if (!reached[i]) {
				if (v[i] <= 0) {
					position[i] = actual_position[i];
				} else {
					position[i] = actual_position[i] + (k[i] * (breaking_node * 0.02 * v[i] - breaking_node * breaking_node * 0.02 * 0.02 * a / 2));
				}
			} //else {
			//	position[i] = actual_position[i];
			//}

			breaking_node++;
		} else {

			position[i] = actual_position[i] + (k[i] * (s[i]/5) * node);
			v[i] = (s[i]/5)/0.02;
			printf("v: %f ", v[i]);
		}
		printf("%f r: %d\t", position[i], reached[i]);
	}
	printf("\n");
	flushall();

	position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position));
	position_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	if(neuron_sensor->current_period==1)
		neuron_sensor->sendCoordinates(position[0],position[1],position[2]);

	for (i = 0; i < 6; i++) {
		if (reached[i] == false) {
			return true;
		}
	}

	printf("return false\n");
	flushall();

	return false;

}

double * neuron_generator::get_position() {
	return position;
}

void neuron_generator::reset() {

	int i;

	breaking = false;

	for (i = 0 ; i < 6; i++) {
		v[i] = 0.0;
	}

	for (i = 0 ; i < 6; i++) {
		a_max[i] = 0.1;
	}

	for (i = 0 ; i < 6; i++) {
		s[i] = 0.0;
	}

	for (i = 0 ; i < 6; i++) {
		k[i] = 0.0;
	}

	for (i = 0 ; i < 6; i++) {
		reached[i] = false;
	}

	breaking_node = 1;
}

void neuron_generator::set_breaking(bool breaking) {
	this->breaking = breaking;
}

}//generator
}//common
}//ecp
}//mrrocpp
