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
#include "ecp_mp/ecp_mp_s_pcbird.h"
#include "ecp_mp/ecp_mp_task.h"



using namespace std;

namespace mrrocpp {
namespace ecp_mp {

/*!
 * Constructor. Creates socket connection to pcbird.
 */
ecp_mp_pcbird_sensor::ecp_mp_pcbird_sensor(SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object)
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set size of passed message/union.
	union_size = sizeof(image.sensor_union.pcbird);

	// Set period variables.
	base_period=current_period=1;

	// Retrieve pcbird node name and port from configuration file.
	int pcbird_port = _ecp_mp_object.config.return_int_value("pcbird_port", _section_name);
	char* pcbird_node_name = _ecp_mp_object.config.return_string_value("pcbird_node_name", _section_name);

  // Try to connect to pcbird.
  if ((sockfd = pcbird_connect(pcbird_node_name, pcbird_port)) == -1)
		throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);

	sr_ecp_msg.message("Connected to pcbird");
}// end: ecp_mp_sensor


/*!
 * Sends sensor configuration to pcbird.
 */
void ecp_mp_pcbird_sensor::configure_sensor() {

  // Start streaming.
/*  if (pcbird_start_streaming(sockfd) == -1)
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);*/


} // end initiate_sensor


/*!
 * Sends initiation reading command to pcbird.
 */
void ecp_mp_pcbird_sensor::initiate_reading() {

}


/*!
 * Retrieves aggregated data from pcbird.
 */
void ecp_mp_pcbird_sensor::get_reading() {

//  pcbird_get_streaming_position(sockfd, (pcbird_pos_t *)&image.sensor_union.pcbird);
  pcbird_get_single_position(sockfd, (pcbird_pos_t *)&image.sensor_union.pcbird);

  printf("[x, y, z] = [%.3f, %.3f, %.3f] ", image.sensor_union.pcbird.x, image.sensor_union.pcbird.y, image.sensor_union.pcbird.z);
  printf("[a, b, g] = [%.3f, %.3f, %.3f] ", image.sensor_union.pcbird.a, image.sensor_union.pcbird.b, image.sensor_union.pcbird.g);
  printf("dist = %.3f ts=%d:%d\n", image.sensor_union.pcbird.distance, image.sensor_union.pcbird.ts_sec, image.sensor_union.pcbird.ts_usec);

}


/*!
 * Closes pcbird socket connection.
 */
void ecp_mp_pcbird_sensor::terminate() {
	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} // end: terminate()

} // namespace ecp_mp
} // namespace mrrocpp
