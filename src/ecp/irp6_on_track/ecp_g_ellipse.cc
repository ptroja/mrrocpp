#include "ecp/irp6_on_track/ecp_g_ellipse.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_g_ellipse::ecp_g_ellipse (ecp_task& _ecp_task,double major_axis,double minor_axis,int max_steps) : ecp_generator (_ecp_task),major_axis(major_axis),minor_axis(minor_axis),d_rad(2*PI/max_steps) {}

bool ecp_g_ellipse::first_step()
{
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = max_steps;
    the_robot->EDP_data.value_in_step_no = max_steps;
    
	step_no = 0;
	
    return true;
}

bool ecp_g_ellipse::next_step()
{
	++step_no;

if(step_no == 1)
{	
	    position[0] = 1; 		//x
    position[1] = 0; 		//y
    position[2] = minor_axis; 	//z
    position[3] = 0; 		//alfa
    position[4] = 1.57; 	//beta
    position[5] = 3.14; 	//gamma
    position[6] = 0.08; 	//G
    position[7] = 0; 		//T

    rad = 0;
}
else
{	
	if(step_no > max_steps)
	{
		return false;
	}
	
	rad += d_rad;
	
    position[0] = 1; 		//x
    position[1] = major_axis*sin(rad); 		//y
    position[2] = minor_axis*cos(rad); 	//z
    position[3] = 0; 		//alfa
    position[4] = 1.57; 	//beta
    position[5] = 3.14; 	//gamma
    position[6] = 0.08; 	//G
    position[7] = 0; 		//T
}
    memcpy(the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates, position,6*sizeof (double));
    the_robot->EDP_data.next_gripper_coordinate = position[6];

	return true;
}
