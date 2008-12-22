/*!
 * \file ecp_t_cvfradia.cc
 * \brief Class responsible for communication with cvFraDIA (testing purposes).
 * - methods definitions.
 * \author tkornuta
 * \date 17.03.2008
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

/*!
 * Initialize task - robot, sensors and generators.
 */
void ecp_task_cvfradia::task_initialization(void)
{
	try
	{
		// Create cvFraDIA sensor - for testing purposes.
		sensor_m[SENSOR_CVFRADIA] = new ecp_mp_cvfradia_sensor(SENSOR_CVFRADIA, "[vsp_cvfradia]", *this, sizeof(sensor_image_t::sensor_union_t::fradia_t));
		// Configure sensor.
		sensor_m[SENSOR_CVFRADIA]->configure_sensor();
    // Create an adequate robot. - depending on the ini section name.
    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new ecp_irp6_on_track_robot (*this);
        sr_ecp_msg->message("IRp6ot loaded");
    }
    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
        ecp_m_robot = new ecp_irp6_postument_robot (*this);
        sr_ecp_msg->message("IRp6p loaded");
    }
		// Create generator and pass sensor to it.
		cvg = new ecp_cvfradia_generator(*this);
	 	cvg->sensor_m = sensor_m;
	}
	catch(...)
	{
		printf("EXCEPTION\n");
	}
}


/*!
 * Main algorithm loop. Retrieves information from cvFraDIA.
 */
void ecp_task_cvfradia::main_task_algorithm(void)
{
	cvg->Move();
}

/*!
 * Returns created task object (Factory Method design pattern).
 */
ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_cvfradia(_config);
}
