#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_wii.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

wii::wii(lib::configurator &_config) : base(_config) {};

void wii::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");

	//create Wii-mote virtual sensor object
	sensor_m[lib::SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(lib::SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(lib::sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[lib::SENSOR_WIIMOTE]->configure_sensor();
}

void wii::main_task_algorithm(void)
{
	double* firstPosition;

    sg = new common::generator::smooth(*this,true);
    eg = new generator::wii(*this);
    
    eg->sensor_m[lib::SENSOR_WIIMOTE] = sensor_m[lib::SENSOR_WIIMOTE];
	firstPosition = eg->getFirstPosition();
	
	sg->reset();
	sg->load_coordinates(lib::XYZ_EULER_ZYZ,firstPosition[0],firstPosition[1],firstPosition[2],firstPosition[3],firstPosition[4],firstPosition[5],firstPosition[6],firstPosition[7]);
	sg->Move();

	while(1)
	{    
    	eg->Move();
    }
    
    ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::wii(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

