#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_wii_velocity.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

wii_velocity::wii_velocity(lib::configurator &_config) : base(_config) {};

void wii_velocity::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");

	//create Wii-mote virtual sensor object
	sensor_m[SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[SENSOR_WIIMOTE]->configure_sensor();
}

void wii_velocity::main_task_algorithm(void)
{
	eg = new generator::wii_velocity(*this);
//	eg = new ecp_tff_nose_run_generator(*this,8);

    eg->sensor_m[SENSOR_WIIMOTE] = sensor_m[SENSOR_WIIMOTE];

    while(1)
    {
        	eg->Move();
    }
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::wii_velocity(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


