/*
 * ecp_g_neuron_generator.cpp
 *
 *  Created on: Jul 2, 2010
 *      Author: tbem
 */

#include "ecp_g_neuron_generator.h"
#include "ecp_mp_neuron_sensor.h"

namespace mrrocpp{
namespace ecp{
namespace common{
namespace generator{

neuron_generator::neuron_generator(common::task::task& _ecp_task):generator(_ecp_task) {

}

neuron_generator::~neuron_generator() {
	// TODO Auto-generated destructor stub
}

bool neuron_generator::first_step(){
	ecp_t.sr_ecp_msg->message("neuron generator first step");
	printf("neuron generator first step\n");
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	//the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
	//the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
	the_robot->ecp_command.instruction.motion_steps = 10;
	the_robot->ecp_command.instruction.value_in_step_no = 10 - 2;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->communicate_with_edp = false;

	neuron_sensor=(ecp_mp::sensor::neuron_sensor*)sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR];
	return true;
}

bool neuron_generator::next_step(){
	the_robot->communicate_with_edp = true;
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	printf("neuron generator next step\n");
	flushall();
	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]= 0;

	}

	/*Check if entire trajectory was already sent, if so, finish the generator*/
	if(neuron_sensor->transmissionFinished()){
		printf("End of transimssion\n");
		return false;
	}

	/*when current_period==0 get_reading is called, so new data is available*/
	if(neuron_sensor->current_period==0){
		printf("x:%lf y:%lf z:%lf\n",neuron_sensor->getCoordinates().x,neuron_sensor->getCoordinates().y,neuron_sensor->getCoordinates().z);
	}

	return true;

}

}//generator
}//common
}//ecp
}//mrrocpp
