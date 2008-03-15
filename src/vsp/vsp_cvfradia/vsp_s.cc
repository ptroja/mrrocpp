/*! \file vsp_cvfradia.cc
 * \brief Class responsible for communication with cvFraDIA framework.
 * - methods definition
 * \author tkornuta
 * \date 15.03.2008
 */

#include <sys/neutrino.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_cvfradia.h"

#include "lib/configurator.h"




extern configurator* config;


/*!
 * Constructor.
 */
vsp_cvfradia::vsp_cvfradia(void)
{
printf("vsp_cvfradia::vsp_cvfradia\n");
	// Set size of passed message/union.
	union_size = sizeof(image.cvFraDIA);

	is_sensor_configured=false;
	is_reading_ready=false;

printf("retrieve data\n");
	// Retrieve cvfradia node name and port from configuration file.
	char* node_name = 	config->return_string_value("cvfradia_node_name");
	int port = config->return_int_value("cvfradia_port");

printf("socket open\n");
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("ERROR opening socket");
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}

printf("get server hostname\n");
	// Get server hostname.
	server = gethostbyname(node_name);
printf("po get server hostname\n");
		if (server == NULL) {
			printf("ERROR, no host %s\n", node_name);
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
printf("reset bias\n");
	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(port);

printf("connect to cvfradia\n");
	// Try to connect to cvFraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		printf("ERROR connecting");
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}
	sr_msg->message("Connected to cvFraDIA");
}

/*!
 * Destructor
 */
vsp_cvfradia::~vsp_cvfradia(void)
{
	close(sockfd);
	printf("Destruktor VSP\n");
}

/*!
 * Sensor configuration.
 */
void vsp_cvfradia::configure_sensor(void)
{
	is_sensor_configured=true;
	sr_msg->message("Sensor initiated");
}
	
/*!
 * Reading initiation and aggregation.
 */
void vsp_cvfradia::initiate_reading(void)
{
printf("vsp_cvfradia::initiate_reading(void)\n");
/*	if (!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);*/

		if (write(sockfd,"ala",3*sizeof(char)) == -1) {
			perror("write() to sockfd error");
			exit(-1);
		}
		
//		bzero(buffer,BUFFER_SIZE);
		
//		int n = read(sockfd,buffer,BUFFER_SIZE);
/*		if (n < 0) {
			perror("read() from socket");
			exit(-1);
		} else if (n == 0) {
			printf("read() from socket returned no data");
			exit(-1);
		}*/
	is_reading_ready=true;
}
		
/*!
 * Retrieval of aggregated sensor data.
 */
void vsp_cvfradia::get_reading(void)
{
	// Check the state of virtual sensor.
/*	if (!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	if (!is_reading_ready)
		throw sensor_error (FATAL_ERROR, READING_NOT_READY);*/

	from_vsp.vsp_report=VSP_REPLY_OK;
	// Copy data to communication buffer.
	from_vsp.comm_image.cvFraDIA.a = 1;
	
	is_reading_ready=false;
}


/*!
 * Function returning object of vsp_cvfradia type - factory method.
 */
vsp_sensor* return_created_sensor(void)
{
	return new vsp_cvfradia();
}
