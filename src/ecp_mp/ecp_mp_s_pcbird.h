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
class pcbird : public lib::sensor
{
private:
	/*!
      * Socket file descriptor.
      */
	int sockfd;

	/*!
      * Link to the SRP communication object.
      */
	lib::sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name.
      */
	lib::SENSOR_ENUM sensor_name;

public:

	/*!
      * Constructor. Creates socket connection to pcbird.
      */
 	pcbird (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object);

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
	~pcbird();

};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif

