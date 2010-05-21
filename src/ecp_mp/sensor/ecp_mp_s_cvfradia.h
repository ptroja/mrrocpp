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
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

template<typename FROM_VSP_T, typename TO_VSP_T = lib::empty_t>
class cvfradia : public fradia_sensor<FROM_VSP_T, TO_VSP_T> {
public:
	cvfradia (lib::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config)
			: fradia_sensor<FROM_VSP_T, TO_VSP_T>(_sensor_name, _section_name, _sr_ecp_msg, config)
	{
	}
};

#if 0
/*!
 * \brief Types commands sent to PW_HaarDetect task.
 */
typedef enum _HD_MODE
{
	WITHOUT_ROTATION, PERFORM_ROTATION
} hd_mode_t;

/*!
 * \class cvfradia
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * \author tkornuta
 */
template <typename SENSOR_IMAGE, typename CONFIGURE_DATA = lib::empty_t>
class cvfradia : public sensor_interface
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
      * Link to the SRP communication object.
      */
	lib::sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name.
      */
	const lib::SENSOR_t sensor_name;

private:
	struct {
		lib::VSP_REPORT_t vsp_report;
		SENSOR_IMAGE comm_image;
	} from_vsp;

public:

	SENSOR_IMAGE image;

	lib::VSP_REPORT_t get_report(void) const
	{
		return from_vsp.vsp_report;
	}

	struct {
		lib::VSP_COMMAND_t i_code;
		CONFIGURE_DATA parameters;
	} to_vsp;

	/*!
      * Constructor. Creates socket connection to cvFraDIA.
      */
 	cvfradia (lib::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config)
		: sr_ecp_msg(_sr_ecp_msg), sensor_name(_sensor_name)
	{
 		// Set period variables.
 		base_period = current_period = 1;

 		// Retrieve cvfradia node name and port from configuration file.
 		uint16_t cvfradia_port = config.value<uint16_t>("cvfradia_port", _section_name);
 		std::string cvfradia_node_name = config.value<std::string>("cvfradia_node_name", _section_name);

 		// Try to open socket.
 		sockfd = socket(AF_INET, SOCK_STREAM, 0);
 		if (sockfd == -1) {
 			printf("ERROR opening socket");
 			throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
 		}

 		// Get server hostname.
 		server = gethostbyname(cvfradia_node_name.c_str());
 		if (server == NULL) {
 			printf("ERROR, no host '%s'\n", cvfradia_node_name.c_str());
 			throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
 		}

 		// Reset socketaddr data.
 		memset(&serv_addr, 0, sizeof(serv_addr));
 		// Fill it with data.
 		serv_addr.sin_family = AF_INET;
 		memcpy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
 		serv_addr.sin_port = htons(cvfradia_port);

 		// Try to establish a connection with cvFraDIA.
 		if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
 			printf("ERROR connecting");
 			throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
 		}

// 		// Retrieve task name.
// 		const std::string task = _ecp_mp_object.config.value<std::string>("cvfradia_task", _section_name);
// 		strcpy(to_vsp.cvfradia_task_name, task.c_str());

 		sr_ecp_msg.message("Connected to cvFraDIA");
 	}

	/*!
      * Sends sensor configuration to cvFraDIA.
      */
	void configure_sensor (void)
	{
		// Send adequate command to cvFraDIA.
		to_vsp.i_code = lib::VSP_CONFIGURE_SENSOR;
		// Name of required task is set in constructor.

		//cout<<to_vsp.cvfradia_task_name;

		if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
		if(read(sockfd, &from_vsp, sizeof(from_vsp))==-1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
		if(from_vsp.vsp_report != lib::VSP_FRADIA_TASK_LOADED){
			printf("void cvfradia::configure_sensor(): Fradia task not found.\n"); fflush(stdout);
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
		}
	}

	/*!
      * Sends initiation reading command to cvFraDIA.
      */
	void initiate_reading (void) {
		// Send adequate command to cvFraDIA.
		to_vsp.i_code = lib::VSP_INITIATE_READING;

		if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}

	/*!
      * Sends given reading command to cvFraDIA.
      */
	void send_reading (const CONFIGURE_DATA & to)
	{
		// Send any command to cvFraDIA.

		if(write(sockfd, &to, sizeof(to)) == -1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}

	/*!
      * Retrieves aggregated data from cvFraDIA.
      */
	void get_reading (void)
	{
		// Send adequate command to cvFraDIA.
		to_vsp.i_code = lib::VSP_GET_READING;
		if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

		// Read aggregated data from cvFraDIA.
	 	if(read(sockfd, &from_vsp, sizeof(from_vsp))==-1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);

		// Check and copy data from buffer to image.
		if(from_vsp.vsp_report == lib::VSP_REPLY_OK)
			image = from_vsp.comm_image;
		else
			sr_ecp_msg.message ("mp_cvfradia: Reply from VSP not ok");
		//cout<<"cvFraDIA: ("<<image.cvFraDIA.x<<","<<image.cvFraDIA.y<<") size: "<<image.cvFraDIA.width<<","<<image.cvFraDIA.height<<")\n";
	}

	/*!
      * Closes cvFraDIA socket connection.
      */
	~cvfradia() {
		// Send adequate command to cvFraDIA.
		to_vsp.i_code = lib::VSP_TERMINATE;
		if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
			throw lib::sensor::sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

		close(sockfd);
		sr_ecp_msg.message("Terminate\n");
	}
};
#endif
} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
