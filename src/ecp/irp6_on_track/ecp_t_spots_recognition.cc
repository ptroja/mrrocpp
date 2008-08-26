/*!
 * \file ecp_t_spots_recognition.cc
 * \brief Piotr Sakowicz's methods responsible
 * for robot moving while external calibration.
 * - methods definitions
 * \author vented.baffle
 * \date 21.08.2008
 */

#include "ecp/irp6_on_track/ecp_t_spots_recognition.h"

//own libraries
//Constructors
ecp_t_spots_recognition::ecp_t_spots_recognition(configurator &_config): ecp_task(_config)
{

}

ecp_t_spots_recognition::~ecp_t_spots_recognition()
{

}

// methods for ECP template to redefine in concrete classes
void ecp_t_spots_recognition::task_initialization(void)
{

	// Create cvFraDIA sensor - for testing purposes.
	sensor_m[SENSOR_CVFRADIA] = new ecp_mp_cvfradia_sensor(SENSOR_CVFRADIA, "[vsp_cvfradia]", *this);
	// Configure sensor.
	sensor_m[SENSOR_CVFRADIA]->configure_sensor();

// Create an adequate robot. - depending on the ini section name.
// Note - only on track working
if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("IRp6 on Track loaded");
}
/*else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
{
    ecp_m_robot = new ecp_irp6_postument_robot (*this);
    sr_ecp_msg->message("IRp6 Postument loaded");
}*/
	// Create generator and pass sensor to it.
	cvg = new ecp_cvfradia_generator(*this);
 	cvg->sensor_m = sensor_m;
}

void ecp_t_spots_recognition::main_task_algorithm(void)
{
	sr_ecp_msg->message("Press START");
	ecp_wait_for_start();

	cvg->Move();

	sr_ecp_msg->message("Press STOP");
	ecp_wait_for_stop();
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_t_spots_recognition(_config);
}
