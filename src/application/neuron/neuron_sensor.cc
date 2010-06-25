/*
 * neuron_sensor.cpp
 *
 *  Created on: Jun 23, 2010
 *      Author: tbem
 */

#include "neuron_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

neuron_sensor::neuron_sensor() {
	a=5;
}

neuron_sensor::~neuron_sensor() {
	// TODO Auto-generated destructor stub
}

void neuron_sensor::get_reading(){
	printf("get reading\n");
}

void neuron_sensor::configure_sensor(){

}

void neuron_sensor::initiate_reading(){
}


} //sensor
} //ecp_mp
} //mrrocpp
