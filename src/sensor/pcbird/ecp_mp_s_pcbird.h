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

#include "ecp_mp/sensor/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * \class ecp_mp_s_pcbird.h
 * \brief Virtual sensor on the ECP/MP side used for communication with pcbird.
 * \author tkornuta
 */

// struktura z pozycja i katami pcbirda
typedef struct _pcbird
{
	float x, y, z; // pozycja
	float a, b, g; // katy (a = azimuth, b = elevation, g = roll)
	float distance; // odleglosc
	uint32_t ts_sec, ts_usec; // timestamp
} pcbird_t;

const lib::SENSOR_t SENSOR_PCBIRD = "SENSOR_PCBIRD";

class pcbird : public sensor_interface
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
      * Sensor name as defined in impconst.h
      */
	const lib::SENSOR_t sensor_name;

public:
	/*!
	 * Data image
	 */
	pcbird_t image;

	/*!
      * Constructor. Creates socket connection to pcbird.
      */
 	pcbird (const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config);

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
