/*! \file ecp_mp_s_cvfradia.cc
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - methods definition
 * \author tkornuta
 * \date 15.03.2008
 */

#include "ecp_mp/ecp_mp_s_cvfradia.h"


/*!
 * Constructor.
 */
ecp_mp_cvfradia_sensor::ecp_mp_cvfradia_sensor(SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object) 
	:ecp_mp_sensor(_sensor_name, _section_name, _ecp_mp_object)
{
	// Set size of passed message/union.
	union_size = sizeof(image.cvFraDIA);
}
