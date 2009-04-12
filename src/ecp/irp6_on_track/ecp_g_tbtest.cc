/*
 * ecp_g_tbtest.cc
 *
 *  Created on: Jan 30, 2009
 *      Author: tbem
 */

#include "ecp/irp6_on_track/ecp_g_tbtest.h"
#include <iostream>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

ecp_g_tbtest::ecp_g_tbtest (common::ecp_task& _ecp_task) : common::ecp_generator (_ecp_task){

}

bool ecp_g_tbtest::first_step()
{
    the_robot->EDP_data.get_type = ARM_DV; // ARM
    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = MOTOR;
    the_robot->EDP_data.get_arm_type = MOTOR;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 6;

    std::cout<<"tb first step"<<std::endl;

    return true;
    //step_no=1;
}

bool ecp_g_tbtest::next_step(){
    double time; //Czas ruchu.

    sensor_m[SENSOR_CVFRADIA]->get_reading();
	if (sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report == VSP_REPLY_OK) {
		std::cout << "Odczyt z fradii: \n"
				<< sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.x
				<< std::endl
				<< sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.y
				<< std::endl;
	}else{
		std::cout<<"code: "<<sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report<<std::endl;
	}

/*


    next_position[0] = 0.899;		//x
    next_position[1] = -0.02;		//y
    next_position[2] = 0.30;		//z
    next_position[3] = -0.115654;
    next_position[4] = 1.38944;
    next_position[5] = 2.35145;
    next_position[6] = 0.074;
    next_position[7] = 0;

    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV; // ARM
    the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = (WORD) ceil(time / STEP);//ceil(tip.motion_time/STEP);
	the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps;

	memcpy(the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates, next_position, 6
			* sizeof(double));
	the_robot->EDP_data.next_gripper_coordinate = next_position[6];
*/
    return true;
}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


