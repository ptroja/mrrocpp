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

#define BUFFER_SIZE 8*256

/*!
 * \class ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * \author tkornuta
 */
class ecp_mp_cvfradia_sensor : public sensor
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

	// Nazwa czujnika.
	char *VSP_NAME;
	
	// Wskaznik na obiekt do komunikacji z SR
	sr_ecp &sr_ecp_msg;

public:

	const SENSOR_ENUM sensor_name; // nazwa czujnika z define w impconst.h  public:
	/*!
      * Constructor.
      */
 	ecp_mp_cvfradia_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object);
	void configure_sensor();
	void initiate_reading();
	void terminate();
	void get_reading();
 	
}; 


#endif
