#include <math.h>

#include "application/wii_ellipse/generator/ecp_g_wii_ellipse.h"
#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

wii_ellipse::wii_ellipse (common::task::task& _ecp_task,double major_axis,double minor_axis,int max_steps) : common::generator::generator (_ecp_task),major_axis(major_axis),minor_axis(minor_axis),max_steps(max_steps),d_rad(2*M_PI/max_steps) {}

bool wii_ellipse::first_step()
{
	/*
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 6;

	step_no = 0;
    rad = 0;
    return true;
    */
}

bool wii_ellipse::next_step()
{
	try
	{
		ecp_mp::sensor::wiimote * wii = dynamic_cast<ecp_mp::sensor::wiimote *>(sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]);
		wii->get_reading();
		char buffer[100];
		if(wii->image.left && !wii->image.right)
		{
			major_axis *= 0.99;
			sprintf(buffer,"Nowa wartosc wiekszej polosi: %.3f",major_axis);
		    sr_ecp_msg.message(buffer);
		}
		if(!wii->image.left && wii->image.right)
		{
			major_axis *= 1.01;
			sprintf(buffer,"Nowa wartosc wiekszej polosi: %.3f",major_axis);
		    sr_ecp_msg.message(buffer);
		}
		if(wii->image.down && !wii->image.up)
		{
			minor_axis *= 0.99;
			sprintf(buffer,"Nowa wartosc mniejszej polosi: %.3f",minor_axis);
		    sr_ecp_msg.message(buffer);
		}
		if(!wii->image.down && wii->image.up)
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
	/*
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = max_steps;
    the_robot->ecp_command.instruction.value_in_step_no = max_steps;
*/

	if(step_no > max_steps)
	{
		//return false;
	}

	rad += d_rad;

    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = 1;
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = major_axis*sin(rad);
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = 0.15+minor_axis*cos(rad);
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = 0.0;
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = 1.57;
    the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = 3.14;
//    the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = 0.08;

	return true;
}

double* wii_ellipse::getFirstPosition()
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

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

