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

void wii_teach::main_task_algorithm(void)
{
    double kw_bok = 0.2;

    sg = new common::generator::smooth2(*this,true);

    sg->set_absolute();
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298, 	   0.200, 		 -0.004, 1.560, -3.141, 0.074, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298+kw_bok, 0.200, 		 -0.004, 1.560, -3.141, 0.074, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298+kw_bok, 0.200+kw_bok, -0.004, 1.560, -3.141, 0.074, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298, 	   0.200+kw_bok, -0.004, 1.560, -3.141, 0.074, 0.000, false);
    sg->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298, 	   0.200, 		 -0.004, 1.560, -3.141, 0.074, 0.000, false);
    sg->Move();


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


