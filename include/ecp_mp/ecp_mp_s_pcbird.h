/*!
 * \file ecp_mp_s_pcbird.h
 * \brief Virtual sensor on the ECP/MP side used for communication with pcbird.
 * - class declaration
 * \author tkornuta
 * \date 24.06.2008
 */

#ifndef __ECP_PCBIRD_H
#define __ECP_PCBIRD_H

#include <netdb.h>

#include "ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * \class ecp_mp_s_pcbird.h
 * \brief Virtual sensor on the ECP/MP side used for communication with pcbird.
 * \author tkornuta
 */
class ecp_mp_pcbird_sensor : public ::sensor
{
private:
	/*!
      * Socket file descriptor.
      */
	int sockfd;

	/*!
      * Link to the SRP communication object.
      */
	sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name.
      */
	SENSOR_ENUM sensor_name; 

public:

	/*!
      * Constructor. Creates socket connection to pcbird.
      */
 	ecp_mp_pcbird_sensor (SENSOR_ENUM _sensor_name, const char* _section_name, task:: ecp_mp_task& _ecp_mp_object);

	/*!
      * Sends sensor configuration to pcbird.
      */
	void configure_sensor (void);	

	/*!
      * Sends initiation reading command to pcbird.
      */
	void initiate_reading (void);

	/*!
      * Retrieves aggregated data from pcbird.
      */
	void get_reading (void);

	/*!
      * Closes pcbird socket connection.
      */
	void terminate();

}; 

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif

