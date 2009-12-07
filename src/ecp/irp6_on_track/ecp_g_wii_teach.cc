#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "ecp_g_wii_teach.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

wii_teach::wii_teach (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : generator (_ecp_task), _wiimote(_wiimote)
{
    int i;
    for(i = 0;i<7;++i) 
    {
        multipliers[i] = 0.003;
        maxChange[i] = 0.0001;
    }
}

bool wii_teach::first_step()
{
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 8;

    step_no = 0;
    releasedA = false;
    stop = false;

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

    int i;
    for(i = 0;i < 7;++i)
    {
        requestedChange[i] = 0;
    }
}

bool wii_teach::calculate_position(void)
{
    int i;
    bool changed = false;
    for(i = 0;i < 7;++i)
    {
        if(fabs(nextChange[i] - requestedChange[i]) < maxChange[i])
        {
            nextChange[i] = requestedChange[i];
        }
        else
        {
            if(requestedChange[i] > nextChange[i]) nextChange[i] = nextChange[i] + maxChange[i];
            else nextChange[i] = nextChange[i] - maxChange[i];
        }
        if(nextChange[i] != 0) changed = true;
    }

    return changed;
}

int wii_teach::get_axis(void)
{
    int axis = -1;
    if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.left)
    {
        axis = 0;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.right)
    {
        axis = 1;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.up)
    {
        axis = 2;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.left)
    {
        axis = 3;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.right)
    {
        axis = 4;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.up)
    {
        axis = 5;
    }
    else if(_wiimote->image.sensor_union.wiimote.down)
    {
        axis = 6;
    }

    return axis;
}

void wii_teach::set_position(void)
{
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[0] = nextChange[0];
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[1] = nextChange[1];
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[2] = nextChange[2];
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[3] = nextChange[3];
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[4] = nextChange[4];
    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[5] = nextChange[5];
    the_robot->EDP_data.next_gripper_coordinate = nextChange[6];
}

bool wii_teach::next_step()
{
    char buffer[200];
    struct lib::ECP_VSP_MSG message;
    int axis;
    double value;
    message.i_code = lib::VSP_CONFIGURE_SENSOR;
    message.wii_command.led_change = false;
    
    try
    {
        if(rumble)
        {
            message.wii_command.rumble = true;
            _wiimote->get_reading(message);
        }
        else
        {
            message.wii_command.rumble = false;
            _wiimote->get_reading(message);
        }
    }
    catch(...)
    {
    }

    if(!_wiimote->image.sensor_union.wiimote.buttonA) releasedA = true;

    ++step_no;
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = 8;
    the_robot->EDP_data.value_in_step_no = 8;

    if(releasedA && _wiimote->image.sensor_union.wiimote.buttonA) stop = true;

    //get value and convert to nonlinear when needed
    value = _wiimote->image.sensor_union.wiimote.orientation_x;
    if(value > -1 && value < 1) value = pow(value,3);

    
    clear_position();
    axis = get_axis();
    if(!stop && axis >= 0)
    {
        requestedChange[axis] = value * multipliers[axis];
    }
    if(!calculate_position() && stop) return false;
    set_position();

    sprintf(buffer,"Moving to: %.5f %.5f %.5f %.5f %.5f %.5f %.5f",
            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[0],
            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[1],
            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[2],
            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[3],
            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[4],
            the_robot->EDP_data.next_XYZ_AA_arm_coordinates[5],
            the_robot->EDP_data.next_gripper_coordinate
            );
    if(false) sr_ecp_msg.message(buffer);

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
                rumble = true;
                break;
            default:
		throw common::ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
		break;

	} /* end: switch */
    }
    else
    {
        rumble = false;
    }
}

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp