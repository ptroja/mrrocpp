/*! \file vsp_cvfradia.h
 * \brief Class responsible for communication with cvFraDIA framework.
 * - class declaration
 * \author tkornuta
 * \date 15.03.2008
 */


#if !defined(_VSP_CVFRADIA_H)
#define _VSP_CVFRADIA_H

#define BUFFER_SIZE 8*256

#include "vsp/vsp_sensor.h"

/*! \class vsp_cvfradia
 * \brief Class responsible for communication with cvFraDIA framework.
 * \author tkornuta
 */
class vsp_cvfradia: public vsp_sensor
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
	hostent *server;

	/*
	 * Buffer used during communication.
	 */
	char buffer[BUFFER_SIZE];
public:
	
	/*!
      * Constructor.
      */
	vsp_cvfradia(void);

	/*!
      * Destructor.
      */
	~vsp_cvfradia(void);

	/*!
      * Sensor configuration.
      */
	void configure_sensor (void);	

	/*!
      * Reading initiation and aggregation.
      */
	void initiate_reading (void);

	/*!
      * Retrieval of aggregated sensor data.
      */
	void get_reading (void);

};

#endif
