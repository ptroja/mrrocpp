/*!
 * \file ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - class declaration
 * \author tkornuta
 * \date 15.03.2008
 */

#ifndef __ECP_CVFRADIA_H
#define __ECP_CVFRADIA_H

#include <netdb.h>

#include "ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {


#define BUFFER_SIZE 8*256

/*!
 * \class ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * \author tkornuta
 */
class cvfradia : public ::sensor
{
private:
	/*!
      * Socket file descriptor.
      */
	int sockfd;

	/*!
      * Server address.
      */
	sockaddr_in serv_addr;

	/*!
      * Pointer to server.
      */
	hostent* server;

	/*!
      * Buffer used by sockets during communication.
      */
	char buffer[BUFFER_SIZE];

	/*!
      * Link to the SRP communication object.
      */
	lib::sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name.
      */
	SENSOR_ENUM sensor_name;

public:

	/*!
      * Constructor. Creates socket connection to cvFraDIA.
      */
 	cvfradia (SENSOR_ENUM _sensor_name, const char* _section_name, task::base& _ecp_mp_object, int _union_size);

	/*!
      * Sends sensor configuration to cvFraDIA.
      */
	void configure_sensor (void);

	/*!
      * Sends initiation reading command to cvFraDIA.
      */
	void initiate_reading (void);

	/*!
      * Sends given reading command to cvFraDIA.
      */
	void send_reading (ECP_VSP_MSG);

	/*!
      * Retrieves aggregated data from cvFraDIA.
      */
	void get_reading (void);

	/*!
      * Closes cvFraDIA socket connection.
      */
	void terminate();

};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
