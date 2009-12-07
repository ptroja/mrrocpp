/*
 * ecp_g_eihcalibration.cc
 *
 *  Created on: July 28, 2009
 *      Author: jkosiore
 */

#include "ecp/common/ecp_g_eihcalibration.h"
#include <cstring>
#include <iostream>
#include <unistd.h>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;


eihgenerator::eihgenerator (common::task::task& _ecp_task)
        : generator (_ecp_task)
{
	count = 0;
}


eihgenerator::~eihgenerator ()
{

}

bool eihgenerator::first_step()
{
	sensor = (ecp_mp::sensor::cvfradia *)sensor_m[lib::SENSOR_CVFRADIA];

	//proste zadanie kinematyki
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = lib::FRAME;

	sensor->to_vsp.i_code = lib::VSP_INITIATE_READING;

	return true;
}

bool eihgenerator::next_step()
{
	float t[12];
	if(sensor->from_vsp.comm_image.sensor_union.chessboard.found == true)
		count++;
	get_frame(t);
	sensor->to_vsp.eihcalibration.frame_number = count;
	for(int i=0; i<12; i++)
	{
		sensor->to_vsp.eihcalibration.transformation_matrix[i] = t[i];
	}
	return false;

}

void eihgenerator::get_frame(float t[12])
{
	std::cout.precision(3);
	std::cout.width(6);
	std::cout.setf(ios::fixed,ios::floatfield);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<4; j++)
		{
			t[4*i+j] = the_robot->EDP_data.current_arm_frame[i][j];
			std::cout << t[4*i+j] << "\t";
		}
		std::cout<<std::endl;
	}
	std::cout<<std::endl;

}


}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


