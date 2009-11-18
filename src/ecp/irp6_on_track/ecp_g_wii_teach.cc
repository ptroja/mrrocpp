#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "ecp_g_wii_teach.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_teach::wii_teach (common::task::task& _ecp_task,lib::sensor* _wiimote) : generator (_ecp_task), _wiimote(_wiimote) {}

bool wii_teach::first_step()
{
    the_robot->EDP_data.instruction_type = lib::GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 8;

    step_no = 0;
    releasedA = 0;

    return true;
}

void wii_teach::clear_position(void)
{
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[0] = 0;
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[1] = 0;
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[2] = 0;
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[3] = 0;
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[4] = 0;
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[5] = 0;
    the_robot->EDP_data.next_gripper_coordinate = 0;
}

bool wii_teach::next_step()
{
    char buffer[200];
    try
    {
    	_wiimote->get_reading();
    }
    catch(...)
    {
    }

    if(!_wiimote->image.sensor_union.wiimote.buttonA) releasedA = 1;

    ++step_no;
    the_robot->EDP_data.instruction_type = lib::SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 8;

    if(releasedA && _wiimote->image.sensor_union.wiimote.buttonA) return false;

    clear_position();

    double value = _wiimote->image.sensor_union.wiimote.orientation_x;
    //convert to nonlinear
    if(value > -1 && value < 1) value = pow(value,3);

    if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.left)
    {
        the_robot->EDP_data.next_XYZ_AA_arm_coordinates[0] = value * 0.003;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.right)
    {
        the_robot->EDP_data.next_XYZ_AA_arm_coordinates[1] = value * 0.003;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.up)
    {
        the_robot->EDP_data.next_XYZ_AA_arm_coordinates[2] = value * 0.003;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.left)
    {
        the_robot->EDP_data.next_XYZ_AA_arm_coordinates[3] = value * 0.003;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.right)
    {
        the_robot->EDP_data.next_XYZ_AA_arm_coordinates[4] = value * 0.003;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.up)
    {
        the_robot->EDP_data.next_XYZ_AA_arm_coordinates[5] = value * 0.003;
    }
    else if(_wiimote->image.sensor_union.wiimote.down)
    {
        the_robot->EDP_data.next_gripper_coordinate = value * 0.003;
    }

//    sprintf(buffer,"Moving to: %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
//            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[0],
//            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[1],
//            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[2],
//            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[3],
//            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[4],
//            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[5],
//            the_robot->EDP_data.next_gripper_coordinate);
//    sr_ecp_msg.message(buffer);

    return true;
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
            case BEYOND_UPPER_LIMIT_AXIS_1:
            case BEYOND_UPPER_LIMIT_AXIS_2:
            case BEYOND_UPPER_LIMIT_AXIS_3:
            case BEYOND_UPPER_LIMIT_AXIS_4:
            case BEYOND_UPPER_LIMIT_AXIS_5:
            case BEYOND_UPPER_LIMIT_AXIS_6:
            case BEYOND_UPPER_LIMIT_AXIS_7:
            case BEYOND_LOWER_D0_LIMIT:
            case BEYOND_LOWER_THETA1_LIMIT:
            case BEYOND_LOWER_THETA2_LIMIT:
            case BEYOND_LOWER_THETA3_LIMIT:
            case BEYOND_LOWER_THETA4_LIMIT:
            case BEYOND_LOWER_THETA5_LIMIT:
            case BEYOND_LOWER_THETA6_LIMIT:
            case BEYOND_LOWER_THETA7_LIMIT:
            case BEYOND_LOWER_LIMIT_AXIS_1:
            case BEYOND_LOWER_LIMIT_AXIS_2:
            case BEYOND_LOWER_LIMIT_AXIS_3:
            case BEYOND_LOWER_LIMIT_AXIS_4:
            case BEYOND_LOWER_LIMIT_AXIS_5:
            case BEYOND_LOWER_LIMIT_AXIS_6:
            case BEYOND_LOWER_LIMIT_AXIS_7:
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