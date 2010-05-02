/*! \file ecp_mp_s_cvfradia.cc
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - methods definition
 * \author tkornuta
 * \date 15.03.2008
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"
#include "ecp_mp/task/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

using namespace std;

/*!
 * Constructor. Creates socket connection to cvFraDIA.
 */
cvfradia::cvfradia(lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object, int _union_size)
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set size of passed message/union.
	union_size = _union_size + sizeof(image.sensor_union.begin);

	// Set period variables.
	base_period=current_period=1;

	// Retrieve cvfradia node name and port from configuration file.
	int cvfradia_port = _ecp_mp_object.config.value<int>("cvfradia_port", _section_name);
	std::string cvfradia_node_name = _ecp_mp_object.config.value<std::string>("cvfradia_node_name", _section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		printf("ERROR opening socket");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Get server hostname.
	server = gethostbyname(cvfradia_node_name.c_str());
	if (server == NULL) {
		printf("ERROR, no host '%s'\n", cvfradia_node_name.c_str());
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Reset socketaddr data.
	memset((char *) &serv_addr, 0, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	memcpy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(cvfradia_port);

	// Try to establish a connection with cvFraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		printf("ERROR connecting");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Retrieve task name.
	std::string task = _ecp_mp_object.config.value<std::string>("cvfradia_task", _section_name);
	strcpy(to_vsp.cvfradia_task_name, task.c_str());

	sr_ecp_msg.message("Connected to cvFraDIA");
}// end: ecp_mp_sensor


/*!
 * Sends sensor configuration to cvFraDIA.
 */
void cvfradia::configure_sensor() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_CONFIGURE_SENSOR;
	// Name of required task is set in constructor.

	//cout<<to_vsp.cvfradia_task_name;

	if(write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	if(read(sockfd, &from_vsp, sizeof(lib::VSP_ECP_MSG))==-1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	if(from_vsp.vsp_report != lib::VSP_FRADIA_TASK_LOADED){
		printf("void cvfradia::configure_sensor(): Fradia task not found.\n"); fflush(stdout);
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	}
}


/*!
 * Sends initiation reading command to cvFraDIA.
 */
void cvfradia::initiate_reading() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_INITIATE_READING;

	if(write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}


/*!
 * Sends given reading command to cvFraDIA.
 */
void cvfradia::send_reading(lib::ECP_VSP_MSG to) {
	// Send any command to cvFraDIA.

	if(write(sockfd, &to, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}


/*!
 * Retrieves aggregated data from cvFraDIA.
 */
void cvfradia::get_reading() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_GET_READING;
	if(write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	// Read aggregated data from cvFraDIA.
 	if(read(sockfd, &from_vsp, sizeof(lib::VSP_ECP_MSG))==-1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);

	// Check and copy data from buffer to image.
	if(from_vsp.vsp_report == lib::VSP_REPLY_OK)
		memcpy( &(image.sensor_union.begin), &(from_vsp.comm_image.sensor_union.begin), union_size);
	else
		sr_ecp_msg.message ("mp_cvfradia: Reply from VSP not ok");
	//cout<<"cvFraDIA: ("<<image.cvFraDIA.x<<","<<image.cvFraDIA.y<<") size: "<<image.cvFraDIA.width<<","<<image.cvFraDIA.height<<")\n";
}


/*!
 * Closes cvFraDIA socket connection.
 */
cvfradia::~cvfradia() {
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_TERMINATE;
	if(write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} // end: terminate()

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

