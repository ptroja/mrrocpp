/**
 * @file
 * @brief Virtual sensor on the ECP/MP side used for communication with PcBird - declaration of the pcbird class.
 *
  * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 * @author tkornuta
 * @date 16.06.2008
 *
 * @ingroup SENSORS PCBIRD_SENSOR
 */

#ifndef __ECP_PCBIRD_H
#define __ECP_PCBIRD_H

#include <netdb.h>

#include "base/ecp_mp/ecp_mp_sensor.h"
#include "sensor/pcbird/birdclient.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {


const lib::sensor::SENSOR_t SENSOR_PCBIRD = "SENSOR_PCBIRD";

/*!
 *
 * @brief Sensor responsible for communication with the PCBird.
 * @author tkornuta
 *
 * @ingroup SENSORS PCBIRD_SENSOR
 */
class pcbird : public ecp_mp::sensor::sensor_interface
{
private:
	/*!
      * @brief Socket file descriptor.
      */
	int sockfd;

	/*!
      *@brief  Link to the SRP communication object.
      */
	lib::sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name as defined in impconst.h
      */
	const lib::sensor::SENSOR_t sensor_name;

public:
	/*!
	 * @brief Data image
	 */
	pcbird_pos_t image;

	/*!
      * @brief Constructor. Creates socket connection to pcbird.
      */
 	pcbird (const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config);

	/*!
      * @brief Sends sensor configuration to pcbird.
      */
	void configure_sensor (void);

	/*!
      * @brief Sends initiation reading command to pcbird.
      */
	void initiate_reading (void);

	/*!
      * @brief Retrieves aggregated data from pcbird.
      */
	void get_reading (void);

	/*!
      * @brief Closes pcbird socket connection.
      */
	~pcbird();

};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
