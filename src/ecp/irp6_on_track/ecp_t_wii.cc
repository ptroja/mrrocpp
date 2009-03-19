#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_wii.h"
#include "lib/mathtr.h"

ecp_task_wii::ecp_task_wii(configurator &_config) : ecp_task(_config) {};

void ecp_task_wii::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");

	//create Wii-mote virtual sensor object
	sensor_m[SENSOR_WIIMOTE] = new ecp_mp_wiimote_sensor(SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[SENSOR_WIIMOTE]->configure_sensor();
}

void ecp_task_wii::main_task_algorithm(void)
{
 	//Polosie elipsy
	double* firstPosition;

    sg = new ecp_smooth_generator(*this,true);
    eg = new ecp_wii_generator(*this);
    firstPosition = eg->getFirstPosition();

	sg->reset();
	sg->load_coordinates(XYZ_EULER_ZYZ,firstPosition[0],firstPosition[1],firstPosition[2],firstPosition[3],firstPosition[4],firstPosition[5],firstPosition[6],firstPosition[7]);
	sg->Move();

    eg->sensor_m[SENSOR_WIIMOTE] = sensor_m[SENSOR_WIIMOTE];
    eg->Move();
    ecp_termination_notice();
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_wii(_config);
}

