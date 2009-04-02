#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_wii_velocity_generator::ecp_wii_velocity_generator (ecp_task& _ecp_task) : ecp_tff_nose_run_generator(_ecp_task)
{



}

bool ecp_wii_velocity_generator::first_step()
{
    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 6;

    return true;
}

bool ecp_wii_velocity_generator::next_step()
{
	char buffer[200];
	try
	{
		sensor_m[SENSOR_WIIMOTE]->get_reading();
	}
	catch(...)
	{
	}

	++step_no;
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 8;



    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 0.85;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = -0.3;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 0.3;
	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] = -1.136 + sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_x;
	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = 1.39 + sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y;
	//the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = 2.3 + sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_z;
	the_robot->EDP_data.next_gripper_coordinate = 0.074;

	return true;
}

