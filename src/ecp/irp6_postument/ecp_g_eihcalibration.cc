/*
 * ecp_g_eihcalibration.cc
 *
 *  Created on: July 28, 2009
 *      Author: jkosiore
 */

#include "ecp/irp6_postument/ecp_g_eihcalibration.h"
#include <cstring>
#include <iostream>
#include <unistd.h>

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace generator {

using namespace std;


eihgenerator::eihgenerator (common::task::task& _ecp_task)
        : generator (_ecp_task)
{

}


eihgenerator::~eihgenerator ()
{

}

bool eihgenerator::first_step()
{
//	sensor = (ecp_mp::sensor::cvfradia *)sensor_m[lib::SENSOR_CVFRADIA];

	//proste zadanie kinematyki
	the_robot->EDP_data.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = lib::FRAME;

/*	//zadanie zrobienia zdjec od fraidii
	sensor->to_vsp.ps_response.command = 38;
	sensor->to_vsp.i_code = lib::VSP_INITIATE_READING;
*/
	return true;
}

bool eihgenerator::next_step()
{
	double t[12];
	get_frame(t);
	for(int i=0; i<12; i++)
	{
		std::cout<<t[i]<<"\t";
		if(i%4==3)
			std::cout<<std::endl;
	}
	std::cout<<std::endl;

}

void eihgenerator::get_frame(double t[12])
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<4; j++)
			t[4*i+j] = the_robot->EDP_data.current_arm_frame[i][j];
	}

}


}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp


