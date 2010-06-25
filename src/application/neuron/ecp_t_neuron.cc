/*
 * ecp_t_neuron.cc
 *
 *  Created on: May 13, 2010
 *      Author: tbem
 */

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_neuron.h"
#include "neuron_sensor.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/*==============================Constructor==================================*/
//Constructors
Neuron::Neuron(lib::configurator &_config): task(_config){
	ecp_m_robot=new irp6ot_m::robot(*this);				//initialization of robot

	//sgen=new common::generator::smooth(*this, true);
	sgen2 = new common::generator::smooth(*this, true);
	sr_ecp_msg->message("ECP loaded Neuron");
};

/*============================Destructor=====================================*/
Neuron::~Neuron(){
	delete ecp_m_robot;
	delete sgen2;
};

/*====================mp_2_ecp_next_state_string_handler=====================*/
void Neuron::mp_2_ecp_next_state_string_handler(void){
	uint8_t choice;
	sr_ecp_msg->message("Game canceled");
	choice=choose_option ("Do you want to play: 1 - Black(blue), or 2 - White(red)", 2);
}

/*=========================main_task_algorithm===============================*/
/*void Neuron::main_task_algorithm(void){
	sr_ecp_msg->message("ECP neuron ready");
	uint8_t choice;

	neuron_sensor* ns = new neuron_sensor();
	ns->get_reading();

	choice=choose_option ("Do you want to play: 1 - Black(blue), or 2 - White(red)", 2);
	if (choice==lib::OPTION_ONE){
		sr_ecp_msg->message("You will play black, while I will play white");
		printf("You will play black, while I will play white\n");
	}else if(choice==lib::OPTION_TWO){
		sr_ecp_msg->message("You will play white, while I will play black");
		printf("You will play white, while I will play black\n");
	}else{
		sr_ecp_msg->message("Game canceled");
	}

	ecp_termination_notice();
};*/


}  //namespace task
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::Neuron(_config);

}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

