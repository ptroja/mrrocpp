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

#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"
#include "ecp_mp/task/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

using namespace std;

/*!
 * Constructor. Creates socket connection to cvFraDIA.
 */
fradia_sensor::fradia_sensor(lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object) :
	sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set size of passed message/union.
	union_size = 0;

	// Set period variables.
	base_period = current_period = 1;

	// Retrieve cvfradia node name and port from configuration file.
	int cvfradia_port = _ecp_mp_object.config.value <int> ("cvfradia_port", _section_name);
	std::string cvfradia_node_name = _ecp_mp_object.config.value <std::string> ("cvfradia_node_name", _section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
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
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *) server->h_addr, (char *) &serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(cvfradia_port);

	// Try to establish a connection with cvFraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		printf("ERROR connecting");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Retrieve task name.
	std::string task = _ecp_mp_object.config.value <std::string> ("cvfradia_task", _section_name).c_str();
	strcpy(to_vsp.cvfradia_task_name, task.c_str());

	reader_thread = new boost::thread(boost::bind(&fradia_sensor::operator(), this));
	sr_ecp_msg.message("Connected to cvFraDIA");
}// end: ecp_mp_sensor


/*!
 * Sends sensor configuration to cvFraDIA.
 */
void fradia_sensor::configure_sensor()
{
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_CONFIGURE_SENSOR;
	// Name of required task is set in constructor.

	//cout<<to_vsp.cvfradia_task_name;

	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}

/*!
 * Sends initiation reading command to cvFraDIA.
 */
void fradia_sensor::initiate_reading()
{
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_INITIATE_READING;

	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}

/*!
 * Sends given reading command to cvFraDIA.
 */
void fradia_sensor::send_reading(lib::ECP_VSP_MSG to)
{
	// Send any command to cvFraDIA.

	if (write(sockfd, &to, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}

/*!
 * Retrieves aggregated data from cvFraDIA.
 */
void fradia_sensor::get_reading()
{
	get_reading_mutex.lock();
	// copy buffers

	memcpy(&image, &(from_vsp_tmp.comm_image), sizeof(lib::SENSOR_IMAGE));

	get_reading_mutex.unlock();
}

/*!
 * Closes cvFraDIA socket connection.
 */
fradia_sensor::~fradia_sensor()
{
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_TERMINATE;
	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} // end: terminate()

void fradia_sensor::operator()()
{
	lib::ECP_VSP_MSG to_vsp_local;
	lib::VSP_ECP_MSG from_vsp_local;

	cout << "void cvfradia::operator()(){" << endl << cout.flush();

	while (1) {
		// TODO: make fradia process blocking queries and then remove this usleep
		usleep(1000);

		// get reading from fradia
		// Send adequate command to cvFraDIA.
		to_vsp_local.i_code = lib::VSP_GET_READING;
		if (write(sockfd, &to_vsp_local, sizeof(lib::ECP_VSP_MSG)) == -1) {
			//TODO: exception
			continue;
			//throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
		}

		// Read aggregated data from cvFraDIA.
		if (read(sockfd, &from_vsp_local, sizeof(lib::VSP_ECP_MSG)) == -1) {
			//TODO: exception
			continue;
			//throw sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
		}

		get_reading_mutex.lock();
		// copy buffers
		memcpy(&from_vsp_tmp, &from_vsp_local, sizeof(lib::VSP_ECP_MSG));

		get_reading_mutex.unlock();
	}
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

