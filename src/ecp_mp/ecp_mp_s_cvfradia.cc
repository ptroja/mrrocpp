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

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp_mp/ecp_mp_task.h"

/*!
 * Constructor.
 */
ecp_mp_cvfradia_sensor::ecp_mp_cvfradia_sensor(SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object) 
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
printf("ecp_mp_cvfradia_sensor::ecp_mp_cvfradia_sensor\n");
	// Set size of passed message/union.
	union_size = sizeof(image.cvFraDIA);

	// Ustawienie domyslnego okresu pracy czujnika.
	base_period=current_period=1;

	node_name = _ecp_mp_object.config.return_string_value("node_name", _section_name);
	VSP_NAME = _ecp_mp_object.config.return_attach_point_name(configurator::CONFIG_SERVER, "resourceman_attach_point", _section_name);
	
	// Retrieve cvfradia node name and port from configuration file.
printf("retrieve data\n");
	int cvfradia_port = _ecp_mp_object.config.return_int_value("cvfradia_port", _section_name);
	char* cvfradia_node_name = _ecp_mp_object.config.return_string_value("cvfradia_node_name", _section_name);
	printf("odczytalem post i nazwe nodu %s:%i\n", cvfradia_node_name, cvfradia_port);
	
	node_name = _ecp_mp_object.config.return_string_value("node_name", _section_name);
	VSP_NAME = _ecp_mp_object.config.return_attach_point_name(configurator::CONFIG_SERVER, "resourceman_attach_point", _section_name);

printf("ecp_mp_cvfradia_sensor::ecp_mp_cvfradia_sensor\n");
	// Set size of passed message/union.
	union_size = sizeof(image.cvFraDIA);


printf("socket open\n");
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("ERROR opening socket");
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}

printf("get server hostname\n");
	// Get server hostname.
	server = gethostbyname(cvfradia_node_name);
printf("po get server hostname\n");
		if (server == NULL) {
			printf("ERROR, no host %s\n", cvfradia_node_name);
			throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
		}
printf("reset socketaddr data\n");
	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(cvfradia_port);

printf("connect to cvfradia\n");
	// Try to connect to cvFraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		printf("ERROR connecting");
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}
	sr_ecp_msg.message("Connected to cvFraDIA");
	

}// end: ecp_mp_sensor

void ecp_mp_cvfradia_sensor::terminate() {
	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} // end: terminate()

void ecp_mp_cvfradia_sensor::configure_sensor() {
	sr_ecp_msg.message("Sensor configured");
}; // end initiate_sensor


void ecp_mp_cvfradia_sensor::initiate_reading() {
printf("ecp_mp_cvfradia_sensor::initiate_reading(void)\n");

		if (write(sockfd,"ala",3*sizeof(char)) == -1) {
		throw sensor_error (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
		
			perror("write() to sockfd error");
		}

};

void ecp_mp_cvfradia_sensor::get_reading() {
printf("ecp_mp_sensor::get_reading()\t");

	bzero(buffer, BUFFER_SIZE);
		
	int n = read(sockfd, buffer, BUFFER_SIZE);
	if (n < 0)
	{
		throw sensor_error (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	} else if (n == 0)
	{
		printf("read() from socket returned no data");
	}
	printf("%s\n",buffer);
	// Check the state of virtual sensor.
/*	if (!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	if (!is_reading_ready)
		throw sensor_error (FATAL_ERROR, READING_NOT_READY);*/

	from_vsp.vsp_report=VSP_REPLY_OK;

};
