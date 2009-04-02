#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_wii_velocity_generator::ecp_wii_velocity_generator (ecp_task& _ecp_task) : ecp_tff_nose_run_generator(_ecp_task)
{
	configure_behaviour(UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION);
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

	configure_velocity (0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
	sprintf(buffer,"Pos: %f %f %f %f %f %f",the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0],the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1],the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2],the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3],the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4],the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5]);
	sr_ecp_msg.message(buffer);

	//the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] = -1.136 + sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_x;
	//the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = 1.39 + sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y;
	//the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = 2.3 + sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_z;
	//the_robot->EDP_data.next_gripper_coordinate = 0.074;

	return true;
}

