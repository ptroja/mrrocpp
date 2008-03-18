/*! \file ecp_mp_s_cvfradia.cc
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - methods definition
 * \author tkornuta
 * \date 15.03.2008
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp_mp/ecp_mp_task.h"

using namespace std;

/*!
 * Constructor. Creates socket connection to cvFraDIA.
 */
ecp_mp_cvfradia_sensor::ecp_mp_cvfradia_sensor(SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object) 
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set size of passed message/union.
	union_size = sizeof(image.cvFraDIA);

	// Set period variables.
	base_period=current_period=1;

	// Retrieve cvfradia node name and port from configuration file.
	int cvfradia_port = _ecp_mp_object.config.return_int_value("cvfradia_port", _section_name);
	char* cvfradia_node_name = _ecp_mp_object.config.return_string_value("cvfradia_node_name", _section_name);
	
	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("ERROR opening socket");
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Get server hostname.
	server = gethostbyname(cvfradia_node_name);
	if (server == NULL) {
		printf("ERROR, no host %s\n", cvfradia_node_name);
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(cvfradia_port);

	// Try to establish a connection with cvFraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		printf("ERROR connecting");
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	sr_ecp_msg.message("Connected to cvFraDIA");
}// end: ecp_mp_sensor


/*!
 * Sends sensor configuration to cvFraDIA.
 */
void ecp_mp_cvfradia_sensor::configure_sensor() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = VSP_CONFIGURE_SENSOR;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}; // end initiate_sensor


/*!
 * Sends initiation reading command to cvFraDIA.
 */
void ecp_mp_cvfradia_sensor::initiate_reading() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = VSP_INITIATE_READING;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
};


/*!
 * Retrieves aggregated data from cvFraDIA.
 */
void ecp_mp_cvfradia_sensor::get_reading() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = VSP_GET_READING;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	// Read aggregated data from cvFraDIA.
 	if(read(sockfd, &from_vsp, sizeof(VSP_ECP_MSG))==-1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);

	// Check and copy data from buffer to image.
	if(from_vsp.vsp_report == VSP_REPLY_OK)
		memcpy( &(image.begin), &(from_vsp.comm_image.begin), union_size);
	else 
		sr_ecp_msg.message ("Reply from VSP not ok");
	cout<<"cvFraDIA: ("<<image.cvFraDIA.x<<","<<image.cvFraDIA.y<<")\n";
};


/*!
 * Closes cvFraDIA socket connection.
 */
void ecp_mp_cvfradia_sensor::terminate() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = VSP_TERMINATE;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} // end: terminate()

