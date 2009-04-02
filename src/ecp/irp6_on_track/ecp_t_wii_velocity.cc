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

ecp_task_wii_velocity::ecp_task_wii_velocity(configurator &_config) : ecp_task(_config) {};

void ecp_task_wii_velocity::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");

	//create Wii-mote virtual sensor object
	sensor_m[SENSOR_WIIMOTE] = new ecp_mp_wiimote_sensor(SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[SENSOR_WIIMOTE]->configure_sensor();
}

void ecp_task_wii_velocity::main_task_algorithm(void)
{

}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_wii_velocity(_config);
}

