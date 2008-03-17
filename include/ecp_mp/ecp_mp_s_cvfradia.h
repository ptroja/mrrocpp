/*!
 * \file ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - class declaration
 * \author tkornuta
 * \date 15.03.2008
 */

#ifndef __ECP_CVFRADIA_H
#define __ECP_CVFRADIA_H

#include "ecp_mp/ecp_mp_sensor.h"


/*!
 * \class ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * \author tkornuta
 */
class ecp_mp_cvfradia_sensor : public ecp_mp_sensor
{
  public:
	/*!
      * Constructor.
      */
 	ecp_mp_cvfradia_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object);
}; 

#endif
