/*
 * ecp_g_robot_calibration.cc
 * For the generators required for robot_calibration project files specifically for tasks ecp_t_robot_calibration.cc
 *  Created on: May05, 2009
 *  Author:	manibaktha
 */

#include "ecp/irp6_on_track/ecp_g_robot_calibration.h"
#include <unistd.h>
#include <iostream.h>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

using namespace std;

robotcalibgen::robotcalibgen (common::task::task& _ecp_task)
        : generator (_ecp_task)
{

}

bool robotcalibgen::first_step()
{
	sr_ecp_msg.message("robotcalibgen::first_step");

/*	the_robot->EDP_data.instruction_type = lib::GET;
    	the_robot->EDP_data.get_type = ARM_DV;
    	the_robot->EDP_data.get_arm_type = lib::XYZ_EULER_ZYZ;
    	the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    	the_robot->EDP_data.next_interpolation_type = lib::MIM;
*/

	// sensor_m represents our virtual sensor (Defns in include/sensor.h and include/ecp/mp/ecp_mp_s_pcbird.h)
//	sensor = (ecp_mp::sensor::pcbird *)sensor_m[lib::SENSOR_PCBIRD];

	/* For pcbird, no need of configure sensor or initiate reading. The constructor of pcbird class 
	   representing the virtual sensor creates a socket connection.Leter we just need to read the data 
	   from pcbird 
	*/
	
	// send the command to get reading from VSP
/*	sensor->to_vsp.i_code = lib::VSP_GET_READING;
	
	//check if the readings are received properly
	if(sensor->from_vsp.vsp_report!= lib::VSP_REPLY_OK)
	{
		// throw error
		sr_ecp_msg.message(" No reply from VSP ");
	
	}
	else
	{
		// pcbird_sensor_image is a union declared in ecp_g_robot_calibration.h to receive readings
		// RHS readings are from VSP internal structures
		//pcbird_sensor_image.sensor_union.pcbird.x = sensor -> from_vsp.comm_image.sensor_union.pcbird.x;
		//pcbird_sensor_image.sensor_union.pcbird.y = sensor -> from_vsp.comm_image.sensor_union.pcbird.y;
		//pcbird_sensor_image.sensor_union.pcbird.z = sensor -> from_vsp.comm_image.sensor_union.pcbird.z;
		//pcbird_sensor_image.sensor_union.pcbird.a = sensor -> from_vsp.comm_image.sensor_union.pcbird.a;
		//pcbird_sensor_image.sensor_union.pcbird.b = sensor -> from_vsp.comm_image.sensor_union.pcbird.b;
		//pcbird_sensor_image.sensor_union.pcbird.g = sensor -> from_vsp.comm_image.sensor_union.pcbird.g;
		//std::cout<<"x from pcbird\n"<<pcbird_sensor_image.sensor_union.pcbird.x;
		//std::cout<<"y from pcbird\n"<<pcbird_sensor_image.sensor_union.pcbird.y;
		//std::cout<<"z from pcbird\n"<<pcbird_sensor_image.sensor_union.pcbird.z;
		//std::cout<<"a from pcbird\n"<<pcbird_sensor_image.sensor_union.pcbird.a;
		//std::cout<<"b from pcbird\n"<<pcbird_sensor_image.sensor_union.pcbird.b;
		//std::cout<<"g from pcbird\n"<<pcbird_sensor_image.sensor_union.pcbird.g;
	}
*/
	iter = 0;
	return true;
}

bool robotcalibgen::next_step()
{
	sr_ecp_msg.message("robotcalibgen::next_step");

/*	if(iter == 0) //first time next_step
	{
		the_robot->EDP_data.instruction_type = lib::SET;
    		the_robot->EDP_data.set_type = ARM_DV;
    		the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
	}*/
	delay(1);
	return false;
}


}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


 
