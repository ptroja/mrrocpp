#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"
#include "application/wii_teach/generator/ecp_g_wii.h"
#include "ecp_g_wii.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

wii::wii (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote) : generator(_ecp_task), _wiimote(_wiimote)
{
    int i;
    for(i  = 0;i < MAX_NO_OF_DEGREES;++i)
    {
        currentValue[i] = 0;
        requestedChange[i] = 0;
        nextChange[i] = 0;
        maxChange[i] = 0;
        multipliers[i] = 0;
    }

    currentGripperValue = 0;
}

bool wii::calculate_change(int axis, double value)
{
    char buffer[200];
    int i;
    bool changed = false;

    value *= multipliers[axis];
    requestedChange[axis] = value;
    
    for(i = 0;i < MAX_NO_OF_DEGREES;++i)
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

int wii::get_axis(void)
{
    int axis = -1;
    if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.left)
    {
        axis = 0;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.up)
    {
        axis = 1;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.right)
    {
        axis = 2;
    }
    else if(!_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.down)
    {
        axis = 3;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.left)
    {
        axis = 4;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.up)
    {
        axis = 5;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.right)
    {
        axis = 6;
    }
    else if(_wiimote->image.sensor_union.wiimote.buttonB && _wiimote->image.sensor_union.wiimote.down)
    {
        axis = 7;
    }

    return axis;
}

bool wii::next_step()
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
 
    if(releasedA && _wiimote->image.sensor_union.wiimote.buttonA) stop = true;

    //get value and convert to nonlinear when needed
    value = _wiimote->image.sensor_union.wiimote.orientation_x;
    if(value > -1 && value < 1) value = pow(value,3);


    preset_position();

    int i;
    for(i = 0;i < 7;++i)
    {
        requestedChange[i] = 0;
    }

    axis = get_axis();
    bool changed = calculate_change(axis,value);
    if(!changed && stop) return false;

    set_position();
    
    return true;
}

void wii::execute_motion(void)
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
