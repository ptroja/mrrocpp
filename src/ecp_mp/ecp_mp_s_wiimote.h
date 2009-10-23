#ifndef __ECP_WIIMOTE_H
#define __ECP_WIIMOTE_H

#include <netdb.h>

#include "ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

#define BUFFER_SIZE 8*256

/**
 * Virtual sensor that communicates with the Wii-mote
 *
 * @author jedrzej
 */
class wiimote : public lib::sensor
{
private:
	//socket descriptor
	int sockfd;

	//address of the wii-mote server
	sockaddr_in serv_addr;

	//pointer to the server
	hostent* server;

	//buffer used for communication
	char buffer[BUFFER_SIZE];

	//link to SRP communication object
	lib::sr_ecp& sr_ecp_msg;

	//sensor name
	lib::SENSOR_ENUM sensor_name;

public:
	/**
	 * Creates the sensor object. Connects to the Wii-mote server
	 *
	 * @param _sensor_name 		the name f the sensor
	 * @param _section_name		the name of the section in the config file
	 * @param _ecp_mp_object
	 * @param _union_size
	 *
	 * @author jedrzej
	 */
	wiimote (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object, int _union_size);

	/**
	 * Sends sensor configuration to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void configure_sensor (void);

	/**
	 * Sends "initiate reading" command to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void initiate_reading (void);

	/**
	 * Sends given command to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void send_reading (lib::ECP_VSP_MSG);

	/**
	 * Retrieves aggregated data from the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void get_reading (void);

	/**
	 * Terminates the connection to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	~wiimote();
};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
