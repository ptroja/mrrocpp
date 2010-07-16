/*
 * ecp_t_neuron.cc
 *
 *  Created on: May 13, 2010
 *      Author: tbem
 */

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_neuron.h"
#include "neuron_sensor.h"
#include "ecp_mp_t_neuron.h"
#include "ecp_mp_neuron_sensor.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/*==============================Constructor==================================*/
//Constructors
Neuron::Neuron(lib::configurator &_config): task(_config){
	//sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR] = new ecp_mp::sensor::neuron_sensor(config);
	ecp_m_robot=new irp6ot_m::robot(*this);				//initialization of robot

	//sgen=new common::generator::smooth(*this, true);
	//sgen2 = new common::generator::smooth(*this, true);
	//neurong = new common::generator::neuron_generator(*this);
	//neurong->sensor_m=sensor_m;
	sr_ecp_msg->message("ECP loaded Neuron");
};

/*============================Destructor=====================================*/
Neuron::~Neuron(){
	delete ecp_m_robot;
	delete neurong;
};

/*====================mp_2_ecp_next_state_string_handler=====================*/
void Neuron::mp_2_ecp_next_state_string_handler(void){
	uint8_t choice;
	if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_T_NEURON) {
		sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR] = new ecp_mp::sensor::neuron_sensor(config);
		neurong = new common::generator::neuron_generator(*this);
		neurong->sensor_m=sensor_m;
		choice=choose_option ("Do you want to play: 1 - Black(blue), or 2 - White(red)", 2);
		sr_ecp_msg->message("Game canceled");

		neurong->Move();
	}


}

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

