#include "ecp/irp6_on_track/ecp_g_ellipse.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ecp_ellipse_generator::ecp_ellipse_generator (ecp_task& _ecp_task,double major_axis,double minor_axis,int max_steps) : ecp_generator (_ecp_task),major_axis(major_axis),minor_axis(minor_axis),max_steps(max_steps),d_rad(2*PI/max_steps) {}

bool ecp_ellipse_generator::first_step()
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
    
	step_no = 0;
    rad = 0;
    return true;
}

bool ecp_ellipse_generator::next_step()
{
	try
	{
		sensor_m[SENSOR_WIIMOTE]->get_reading();
		char buffer[100];
		if(sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.left && !sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.right)
		{
			major_axis *= 0.99;
			sprintf(buffer,"Nowa wartosc wiekszej polosi: %.3f",major_axis);
		    sr_ecp_msg.message(buffer);
		}
		if(!sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.left && sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.right)
		{
			major_axis *= 1.01;
			sprintf(buffer,"Nowa wartosc wiekszej polosi: %.3f",major_axis);
		    sr_ecp_msg.message(buffer);
		}
		if(sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.down && !sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.up)
		{
			minor_axis *= 0.99;
			sprintf(buffer,"Nowa wartosc mniejszej polosi: %.3f",minor_axis);
		    sr_ecp_msg.message(buffer);
		}
		if(!sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.down && sensor_m[SENSOR_WIIMOTE]->image.sensor_union.wiimote.up)
		{
			minor_axis *= 1.01;
			sprintf(buffer,"Nowa wartosc mniejszej polosi: %.3f",minor_axis);
		    sr_ecp_msg.message(buffer);
		}
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
    the_robot->EDP_data.motion_steps = max_steps;
    the_robot->EDP_data.value_in_step_no = max_steps;


	if(step_no > max_steps)
	{
		//return false;
	}
	
	rad += d_rad;
	
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 1;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = major_axis*sin(rad);
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 0.15+minor_axis*cos(rad);
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] = 0.0;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = 1.57;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = 3.14;
    the_robot->EDP_data.next_gripper_coordinate = 0.08;

	return true;
}

double* ecp_ellipse_generator::getFirstPosition()
{
	double* firstPosition = new double[8];
	firstPosition[0] = 1;
	firstPosition[1] = 0;//major_axis*sin(rad);
	firstPosition[2] = 0.15 + minor_axis;//*cos(rad);
	firstPosition[3] = 0.0;
	firstPosition[4] = 1.57;
	firstPosition[5] = 3.14;
	firstPosition[6] = 0.08;
	
	return firstPosition;
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

