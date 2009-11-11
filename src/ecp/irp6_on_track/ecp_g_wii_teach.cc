#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "ecp_g_wii_teach.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_teach::wii_teach (common::task::task& _ecp_task) : generator (_ecp_task) {}

bool wii_teach::first_step()
{
    the_robot->EDP_data.instruction_type = lib::GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 6;

    step_no = 0;
    rad = 0;
    return true;
}

bool wii_teach::next_step()
{
    char buffer[200];
    try
    {
    	sensor_m[lib::SENSOR_WIIMOTE]->get_reading();
    }
    catch(...)
    {
    }

    ++step_no;
    the_robot->EDP_data.instruction_type = lib::SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 8;


    if(sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonB)
    {
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 0.92;
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = 0;
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 0.27;
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] = -1.136 + sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_x;
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = 1.39 + sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y;
        the_robot->EDP_data.next_gripper_coordinate = 0.074;
        return true;
    }

    return false;
}

double* wii_teach::getFirstPosition()
{

    double* firstPosition = new double[8];
    firstPosition[0] = 0.92;
    firstPosition[1] = 0;
    firstPosition[2] = 0.27;

    firstPosition[3] = -1.136 + sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_x;
    firstPosition[4] = 1.38 + sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y;
    firstPosition[5] = 2.3;
    firstPosition[6] = 0.074;

    return firstPosition;
}

void wii_teach::execute_motion(void)
{
    // komunikacja wlasciwa
    the_robot->send();
    if (the_robot->reply_package.reply_type == lib::ERROR)
    {
	the_robot->query();
    	throw common::ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
    }
    the_robot->query();

    if (the_robot->reply_package.reply_type == lib::ERROR)
    {
	switch ( the_robot->reply_package.error_no.error0 )
        {
            case BEYOND_UPPER_D0_LIMIT:
            case BEYOND_UPPER_THETA1_LIMIT:
            case BEYOND_UPPER_THETA2_LIMIT:
            case BEYOND_UPPER_THETA3_LIMIT:
            case BEYOND_UPPER_THETA4_LIMIT:
            case BEYOND_UPPER_THETA5_LIMIT:
            case BEYOND_UPPER_THETA6_LIMIT:
            case BEYOND_UPPER_THETA7_LIMIT:
            case BEYOND_LOWER_D0_LIMIT:
            case BEYOND_LOWER_THETA1_LIMIT:
            case BEYOND_LOWER_THETA2_LIMIT:
            case BEYOND_LOWER_THETA3_LIMIT:
            case BEYOND_LOWER_THETA4_LIMIT:
            case BEYOND_LOWER_THETA5_LIMIT:
            case BEYOND_LOWER_THETA6_LIMIT:
            case BEYOND_LOWER_THETA7_LIMIT:
                break;
            default:
		throw common::ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
		break;

	} /* end: switch */
    }
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp