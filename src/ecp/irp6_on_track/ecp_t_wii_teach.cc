#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_wii_teach.h"
#include "lib/mathtr.h"
#include "ecp_t_wii_teach.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

wii_teach::wii_teach(lib::configurator &_config) : task(_config)
{
    ecp_m_robot = new robot (*this);

    //create Wii-mote virtual sensor object
    sensor_m[lib::SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(lib::SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(lib::sensor_image_t::sensor_union_t::wiimote_t));
    //configure the sensor
    sensor_m[lib::SENSOR_WIIMOTE]->configure_sensor();
}

void wii_teach::updateButtonsPressed(void)
{
    buttonsPressed.left = !lastButtons.left && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.left;
    buttonsPressed.right = !lastButtons.right && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right;
    buttonsPressed.up = !lastButtons.up && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.up;
    buttonsPressed.down = !lastButtons.down && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.down;
    buttonsPressed.buttonA = !lastButtons.buttonA && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonA;
    buttonsPressed.buttonB = !lastButtons.buttonB && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonB;
    buttonsPressed.button1 = !lastButtons.button1 && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.button1;
    buttonsPressed.button2 = !lastButtons.button2 && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.button2;
    buttonsPressed.buttonPlus = !lastButtons.buttonPlus && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonPlus;
    buttonsPressed.buttonMinus = !lastButtons.buttonMinus && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonMinus;
    buttonsPressed.buttonHome = !lastButtons.buttonHome && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonHome;
}

void wii_teach::main_task_algorithm(void)
{
    double kw_bok = 0.2;
    int s = 0;
    int m = 0;
    int mv = 0;
    int pos = 0;
    int mode = 0;
    char buffer[80];

    sg = new common::generator::smooth2(*this,true);
    wg = new irp6ot::generator::wii_teach(*this);

    sg->set_absolute();
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, -0.1, 0.27, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, 0.1, 0.27, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, 0.1, 0.4, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, -0.1, 0.4, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, -0.1, 0.27, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, 0.1, 0.27, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, 0.1, 0.4, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.92, -0.1, 0.4, 0, 1.570, -3.141, 0.08, 0.000, false);
    sg->Move();

    while(1)
    {
        sensor_m[lib::SENSOR_WIIMOTE]->get_reading();
        updateButtonsPressed();
        lastButtons = sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote;

        if(sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonB)
        {
            sprintf(buffer,"RUCH %d",++m);
            sr_ecp_msg->message(buffer);
            wg->Move();
        }
        else
        {
            sprintf(buffer,"CZEKAM %d",++s);
            sr_ecp_msg->message(buffer);

            if(buttonsPressed.button1)
            {
                buttonsPressed.button1 = 0;
                mode = 1;
                sr_ecp_msg->message("mode add");
            }
            else if(buttonsPressed.button2)
            {
                buttonsPressed.button2 = 0;
                mode = 2;
                sr_ecp_msg->message("mode edit");
            }
            else if(buttonsPressed.buttonMinus)
            {
                buttonsPressed.buttonMinus = 0;
                sr_ecp_msg->message("1 step back");
                if(pos > 1) --pos;
                sprintf(buffer,"pos %d",pos);
                sr_ecp_msg->message(buffer);
            }
            else if(buttonsPressed.buttonPlus)
            {
                buttonsPressed.buttonPlus = 0;
                sr_ecp_msg->message("1 step forward");
                if(pos < mv) ++pos;
                sprintf(buffer,"pos %d",pos);
                sr_ecp_msg->message(buffer);
            }
            else if(buttonsPressed.buttonHome)
            {
                buttonsPressed.buttonPlus = 0;
                if(mv > 0)
                {
                    sr_ecp_msg->message("start");
                    pos = 1;
                    sprintf(buffer,"pos %d",pos);
                    sr_ecp_msg->message(buffer);
                }
                else
                {
                    sr_ecp_msg->message("add points first");
                }
                
            }
            else if(buttonsPressed.buttonA)
            {
                buttonsPressed.buttonPlus = 0;
                if(mode == 1)
                {
                    ++mv;
                    sprintf(buffer,"add point %d, have %d",pos,mv);
                }
                else
                {
                    sprintf(buffer,"edit point %d",pos);
                }
                sr_ecp_msg->message(buffer);
            }
        }

    }

    ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::wii_teach(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


