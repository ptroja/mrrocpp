/*!
 * \file ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - class declaration
 * \author tkornuta
 * \date 15.03.2008
 */

#ifndef __FRADIA_SENSOR_BLOCKING_H
#define __FRADIA_SENSOR_BLOCKING_H

#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp_mp/task/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

//#define BUFFER_SIZE 8*256

template <typename VSP_ECP_T>
class fradia_sensor : public lib::sensor
{
public:

	/*!
      * Constructor. Creates socket connection to cvFraDIA.
      */
	fradia_sensor (const char* _section_name, task::task& _ecp_mp_object);

	/*!
      * Sends sensor configuration to cvFraDIA.
      */
	void configure_sensor (void);

	/*!
      * Does nothing.
      */
	void initiate_reading (void){}

	/*!
      * Sends given reading command to cvFraDIA.
      */
	void send_reading (lib::ECP_VSP_MSG);

	/*!
      * Retrieves aggregated data from cvFraDIA.
      */
	void get_reading (void);

	/*!
      * Closes cvFraDIA socket connection.
      */
	~fradia_sensor();

	void operator()();


	VSP_ECP_T received_object;
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
      * Buffer used by sockets during communication.
      */
	//char buffer[BUFFER_SIZE];

	/*!
      * Link to the SRP communication object.
      */
	lib::sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name.
      */
	//const lib::SENSOR_t sensor_name;

	lib::ECP_VSP_MSG to_vsp_tmp;

	lib::VSP_ECP_MSG from_vsp_tmp;
	/*!
	 * Thread which continously read measurments from fradia over TCP/IP socket.
	 * Measurements are buffered and always available to get_reading().
	 */
	boost::thread* reader_thread;

	/**
	 * Mutex for from_vsp_tmp.
	 */
	boost::mutex get_reading_mutex;

	VSP_ECP_T received_object_tmp;
}; // class fradia_sensor

template <typename VSP_ECP_T>
fradia_sensor<VSP_ECP_T>::fradia_sensor(const char* _section_name, task::task& _ecp_mp_object) :
	sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg)
{
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

	memset(&received_object, 0, sizeof(VSP_ECP_T));
	memset(&received_object_tmp, 0, sizeof(VSP_ECP_T));

	//reader_thread = new boost::thread(boost::bind(&fradia_sensor<VSP_ECP_T>::operator(), this));
	reader_thread = NULL;
	sr_ecp_msg.message("Connected to cvFraDIA");
}

template <typename VSP_ECP_T>
void fradia_sensor<VSP_ECP_T>::configure_sensor()
{
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_CONFIGURE_SENSOR;
	// Name of required task is set in constructor.

	//cout<<to_vsp.cvfradia_task_name;

	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	reader_thread = new boost::thread(boost::bind(&fradia_sensor<VSP_ECP_T>::operator(), this));

	//printf("fradia_sensor<VSP_ECP_T>::configure_sensor() end\n"); fflush(stdout);
}

template <typename VSP_ECP_T>
void fradia_sensor<VSP_ECP_T>::send_reading(lib::ECP_VSP_MSG to)
{
	// Send any command to cvFraDIA.

	if (write(sockfd, &to, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}

template <typename VSP_ECP_T>
void fradia_sensor<VSP_ECP_T>::get_reading()
{
	get_reading_mutex.lock();
	// copy buffers

	memcpy(&received_object, &received_object_tmp, sizeof(VSP_ECP_T));

	get_reading_mutex.unlock();

	//printf("fradia_sensor::get_reading()\n");
}

template <typename VSP_ECP_T>
fradia_sensor<VSP_ECP_T>::~fradia_sensor()
{
	// Send adequate command to cvFraDIA.
	to_vsp.i_code = lib::VSP_TERMINATE;
	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
} // end: terminate()

template <typename VSP_ECP_T>
void fradia_sensor<VSP_ECP_T>::operator()()
{
	int result;
	lib::ECP_VSP_MSG to_vsp_local;
	lib::VSP_ECP_MSG from_vsp_local;

	//printf("fradia_sensor::operator() begin\n"); fflush(stdout);

	while (1) {
		// TODO: make fradia process blocking queries and then remove this usleep
		usleep(10000);

		// get reading from fradia
		// Send adequate command to cvFraDIA.
		to_vsp_local = to_vsp;
		to_vsp_local.i_code = lib::VSP_GET_READING;
		result = write(sockfd, &to_vsp_local, sizeof(lib::ECP_VSP_MSG));
		if (result < 0) {
			//TODO: exception
			printf("write() error.\n"); fflush(stdout);
			continue;
			//throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
		}
		if(result != sizeof(lib::ECP_VSP_MSG)){
			printf("result != sizeof(lib::ECP_VSP_MSG)\n"); fflush(stdout);
			continue;
		}

		// Read aggregated data from cvFraDIA.
		result = read(sockfd, &from_vsp_local, sizeof(lib::VSP_ECP_MSG));
		if (result < 0) {
			//TODO: exception
			printf("read() error.\n"); fflush(stdout);
			continue;
			//throw sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
		}
		if(result != sizeof(lib::VSP_ECP_MSG)){
			printf("result != sizeof(lib::VSP_ECP_MSG)\n"); fflush(stdout);
			continue;
		}

		if(from_vsp_local.vsp_report != lib::VSP_REPLY_OK){
			printf("(from_vsp_local.vsp_report != lib::VSP_REPLY_OK\n"); fflush(stdout);
			continue;
		}

		get_reading_mutex.lock();
		// copy buffers
		memcpy(&received_object_tmp, &from_vsp_local.comm_image.sensor_union.begin, sizeof(VSP_ECP_T));

		get_reading_mutex.unlock();

		//printf("fradia_sensor::operator()\n"); fflush(stdout);
	}
}


} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
