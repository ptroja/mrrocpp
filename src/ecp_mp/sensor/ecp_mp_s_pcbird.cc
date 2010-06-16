/*! \file ecp_mp_s_pcbird.cc
 * \brief Virtual sensor on the ECP/MP side used for communication with pcbird framework.
 * - methods definition
 * \author tkornuta
 * \date 15.03.2008
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>

#include "ecp_mp/birdclient.h"
#include "ecp_mp/sensor/ecp_mp_s_pcbird.h"
#include "ecp_mp/task/ecp_mp_task.h"



using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * Constructor. Creates socket connection to pcbird.
 */
pcbird::pcbird(const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config)
	: sr_ecp_msg(_sr_ecp_msg), sensor_name(SENSOR_PCBIRD)
{
	// Set period variables.
	base_period = current_period = 1;

	// Retrieve pcbird node name and port from configuration file.
	int pcbird_port = config.value<int>("pcbird_port", _section_name);
	std::string pcbird_node_name = config.value<std::string>("pcbird_node_name", _section_name);

	// Try to connect to pcbird.
	if ((sockfd = pcbird_connect(pcbird_node_name.c_str(), pcbird_port)) == -1)
		throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);

	sr_ecp_msg.message("Connected to pcbird");
}


/*!
 * Sends sensor configuration to pcbird.
 */
void pcbird::configure_sensor() {

  // Start streaming.
/*  if (pcbird_start_streaming(sockfd) == -1)
		throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);*/


}


/*!
 * Sends initiation reading command to pcbird.
 */
void pcbird::initiate_reading() {
}


/*!
 * Retrieves aggregated data from pcbird.
 */
void pcbird::get_reading() {
//	sr_ecp_msg.message("PCBIRD: before get_reading");

//  pcbird_get_streaming_position(sockfd, (pcbird_pos_t *)&image.sensor_union.pcbird);
  pcbird_get_single_position(sockfd, (pcbird_pos_t*) &image);

/*
  printf("[x, y, z] = [%.3f, %.3f, %.3f] ", image.x, image.y, image.z);
  printf("[a, b, g] = [%.3f, %.3f, %.3f] ", image.a, image.b, image.g);
  printf("dist = %.3f ts=%d:%d\n", image.distance, image.ts_sec, image.ts_usec);
*/

/*	char measures[80];
	sprintf(measures, "PCBIRD [x, y, z, a, b, g] = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] ", image.x, image.y, image.z, image.a, image.b, image.g);
	//sprintf(measures, "PCBIRD [x, y, z]");
	sr_ecp_msg.message(measures);
*/
}


/*!
 * Closes pcbird socket connection.
 */
pcbird::~pcbird() {
	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
