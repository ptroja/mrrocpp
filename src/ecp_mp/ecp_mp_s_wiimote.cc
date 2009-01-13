#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>

#include "ecp_mp/ecp_mp_s_wiimote.h"
#include "ecp_mp/ecp_mp_task.h"

using namespace std;

ecp_mp_wiimote_sensor::ecp_mp_wiimote_sensor(SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object, int _union_size)
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set size of passed message/union.
	union_size = _union_size + sizeof(image.sensor_union.begin);

	// Set period variables.
	base_period=current_period=1;

	// Retrieve wiimote node name and port from configuration file.
	int wiimote_port = _ecp_mp_object.config.return_int_value("wiimote_port", _section_name);
	char* wiimote_node_name = _ecp_mp_object.config.return_string_value("wiimote_node_name", _section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		sr_ecp_msg.message("ERROR opening socket");
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}
    
	// Get server hostname.
	server = gethostbyname(wiimote_node_name);
	if (server == NULL) {
		sprintf(buffer,"ERROR, no host %s", wiimote_node_name);
		sr_ecp_msg.message(buffer);
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(wiimote_port);

	// Try to establish a connection with wiimote.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		sr_ecp_msg.message("Error connecting");
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	sr_ecp_msg.message("Connected to wiimote");
}


void ecp_mp_wiimote_sensor::configure_sensor() {
	// Send adequate command to wiimote.
	to_vsp.i_code = VSP_CONFIGURE_SENSOR;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}


void ecp_mp_wiimote_sensor::initiate_reading() {
	// Send adequate command to wiimote.
	to_vsp.i_code = VSP_INITIATE_READING;

	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}


void ecp_mp_wiimote_sensor::send_reading(ECP_VSP_MSG to) {
	// Send any command to wiimote.

	if(write(sockfd, &to, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}


void ecp_mp_wiimote_sensor::get_reading() {
	// Send adequate command to wiimote.
	to_vsp.i_code = VSP_GET_READING;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	// Read aggregated data from wiimote.
 	if(read(sockfd, &from_vsp, sizeof(VSP_ECP_MSG))==-1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);

	// Check and copy data from buffer to image.
	if(from_vsp.vsp_report == VSP_REPLY_OK)
		memcpy( &(image.sensor_union.begin), &(from_vsp.comm_image.sensor_union.begin), union_size);
	else
		sr_ecp_msg.message ("Reply from VSP not ok");
}


void ecp_mp_wiimote_sensor::terminate() {
	// Send adequate command to wiimote.
	to_vsp.i_code = VSP_TERMINATE;
	if(write(sockfd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} 

