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
#include <cstring>
#include <strings.h>

#include <cstdio>

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp_mp/task/ecp_mp_task.h"

#include "application/servovision/logger.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * Class for communication with FraDIA. Parametrized by received structure.
 * make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
 * DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
 */
template <typename FROM_VSP_T, typename TO_VSP_T>
class fradia_sensor: public lib::sensor
{
public:

	/*!
	 * Constructor. Creates socket connection to cvFraDIA.
	 */
	fradia_sensor(const char* section_name, mrrocpp::lib::configurator& configurator);

	/*!
	 * Closes cvFraDIA socket connection.
	 */
	~fradia_sensor();

	/*!
	 * Sends sensor configuration to cvFraDIA.
	 */
	void configure_sensor(void);

	/*!
	 * Does nothing.
	 */
	void initiate_reading(void)
	{
	}

	/*!
	 * Retrieves aggregated data from cvFraDIA.
	 */
	void get_reading(void);

	void configure_fradia_task(const TO_VSP_T& object_to_send);
	//FROM_VSP_T getReceivedObject();

	/*!
	 * MRROC++ <-> FraDIA communication thread method. Implements loop which queries FraDIA for new reading.
	 * make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
	 * DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
	 */
	void operator()();

	FROM_VSP_T received_object;
private:
	std::string fradia_task;
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

	/** Configurator. */
	mrrocpp::lib::configurator& configurator;

	lib::ECP_VSP_MSG to_vsp_tmp;

	lib::VSP_ECP_MSG from_vsp_tmp;
	/*!
	 * Thread which continously read measurments from fradia over TCP/IP socket.
	 * Measurements are buffered and always available to get_reading().
	 */
	boost::shared_ptr <boost::thread> reader_thread;

	/**
	 * Mutex for from_vsp_tmp.
	 */
	boost::mutex get_reading_mutex;

	boost::mutex object_to_send_mutex;

	FROM_VSP_T received_object_shared;
	volatile bool is_object_to_send_set;
	TO_VSP_T object_to_send_shared;

	void send_to_vsp(const lib::ECP_VSP_MSG& ecp_vsp_msg);
	lib::VSP_ECP_MSG receive_from_vsp();
	volatile bool terminate_now;
}; // class fradia_sensor

template <typename FROM_VSP_T, typename TO_VSP_T>
fradia_sensor <FROM_VSP_T, TO_VSP_T>::fradia_sensor(const char* section_name, mrrocpp::lib::configurator& configurator) :
	configurator(configurator), is_object_to_send_set(false), terminate_now(false)
{
	if (sizeof(FROM_VSP_T) > SENSOR_IMAGE_FRADIA_READING_SIZE) {
		logger::logDbg("sizeof(FROM_VSP_T): %d\n", (int) sizeof(FROM_VSP_T));
		logger::logDbg("SENSOR_IMAGE_FRADIA_READING_SIZE: %d\n", SENSOR_IMAGE_FRADIA_READING_SIZE);
		throw std::logic_error("sizeof(FROM_VSP_T) > sizeof(SENSOR_IMAGE)");
	}
	if (sizeof(TO_VSP_T) > ECP_VSP_MSG_FRADIA_COMMAND_SIZE) {
		logger::logDbg("sizeof(TO_VSP_T): %d\n", (int) sizeof(TO_VSP_T));
		logger::logDbg("ECP_VSP_MSG_FRADIA_COMMAND_SIZE: %d\n", ECP_VSP_MSG_FRADIA_COMMAND_SIZE);
		throw std::logic_error("sizeof(TO_VSP_T) > ECP_VSP_MSG_FRADIA_SENSOR_COMMAND_SIZE");
	}

	logger::logDbg("sizeof(lib::VSP_ECP_MSG): %d\n", (int) sizeof(lib::VSP_ECP_MSG));
	logger::logDbg("sizeof(lib::ECP_VSP_MSG): %d\n", (int) sizeof(lib::ECP_VSP_MSG));

	// Set period variables.
	base_period = current_period = 1;

	// Retrieve cvfradia node name and port from configuration file.
	int cvfradia_port = configurator.value <int> ("fradia_port", section_name);
	std::string cvfradia_node_name = configurator.value <std::string> ("fradia_node_name", section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		logger::log("ERROR opening socket");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Get server hostname.
	server = gethostbyname(cvfradia_node_name.c_str());
	if (server == NULL) {
		logger::log("ERROR, no host '%s'\n", cvfradia_node_name.c_str());
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
		logger::log("ERROR connecting");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	// Retrieve task name.
	fradia_task = configurator.value <std::string> ("fradia_task", section_name);

	std::memset(&received_object, 0, sizeof(FROM_VSP_T));
	std::memset(&received_object_shared, 0, sizeof(FROM_VSP_T));
}

template <typename FROM_VSP_T, typename TO_VSP_T>
fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor()
{
	logger::logDbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor() begin\n");
	terminate_now = true;
	//	lib::ECP_VSP_MSG ecp_vsp_msg;
	//	ecp_vsp_msg.i_code = lib::VSP_TERMINATE;
	//	send_to_vsp(ecp_vsp_msg);

	if (reader_thread.get()) {
		reader_thread->join();
	}
	close(sockfd);
	logger::logDbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor() end\n");
} // end: terminate()

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::send_to_vsp(const lib::ECP_VSP_MSG& ecp_vsp_msg)
{
	int result;
	result = write(sockfd, &ecp_vsp_msg, sizeof(lib::ECP_VSP_MSG));
	if (result < 0) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): write failed\n");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
	if (result != sizeof(lib::ECP_VSP_MSG)) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): result != sizeof(lib::ECP_VSP_MSG)\n");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
}

template <typename FROM_VSP_T, typename TO_VSP_T>
lib::VSP_ECP_MSG fradia_sensor <FROM_VSP_T, TO_VSP_T>::receive_from_vsp()
{
	int result;
	lib::VSP_ECP_MSG vsp_ecp_msg;

	result = read(sockfd, &vsp_ecp_msg, sizeof(lib::VSP_ECP_MSG));
	if (result < 0) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): read failed\n");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	}
	if (result != sizeof(lib::VSP_ECP_MSG)) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): result != sizeof(lib::VSP_ECP_MSG)\n");
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	}
	return vsp_ecp_msg;
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor()
{
	lib::ECP_VSP_MSG ecp_vsp_msg;
	lib::VSP_ECP_MSG vsp_ecp_msg;

	// Send adequate command to cvFraDIA.
	ecp_vsp_msg.i_code = lib::VSP_CONFIGURE_SENSOR;
	// Name of required task is set in constructor.
	strcpy(ecp_vsp_msg.cvfradia_task_name, fradia_task.c_str());

	send_to_vsp(ecp_vsp_msg);
	vsp_ecp_msg = receive_from_vsp();

	if (vsp_ecp_msg.vsp_report == lib::VSP_FRADIA_TASK_LOADED) {
		reader_thread
				= boost::shared_ptr <boost::thread>(new boost::thread(boost::bind(&fradia_sensor<FROM_VSP_T, TO_VSP_T>::operator(), this)));
	} else {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): can not load FraDIA task: %s.\n", vsp_ecp_msg.comm_image.sensor_union.fradia_sensor_reading);
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	logger::logDbg("fradia_sensor<FROM_VSP_T>::configure_sensor() end\n");
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::get_reading()
{
	get_reading_mutex.lock();

	received_object = received_object_shared;

	get_reading_mutex.unlock();

	//logger::logDbg("fradia_sensor::get_reading()\n");
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_fradia_task(const TO_VSP_T& object_to_send)
{
	object_to_send_mutex.lock();

	logger::logDbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_fradia_task()\n");
	object_to_send_shared = object_to_send;

	is_object_to_send_set = true;

	object_to_send_mutex.unlock();
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()()
{
	//	int result;
	//	lib::ECP_VSP_MSG to_vsp_local;
	//	lib::VSP_ECP_MSG from_vsp_local;

	lib::ECP_VSP_MSG ecp_vsp_msg;
	lib::VSP_ECP_MSG vsp_ecp_msg;
	TO_VSP_T object_to_send;
	FROM_VSP_T received_object;

	memset(ecp_vsp_msg.fradia_sensor_command, 0, ECP_VSP_MSG_FRADIA_COMMAND_SIZE);

	int iter = 0;
	logger::logDbg("fradia_sensor::operator() begin\n");

	try {
		while (!terminate_now) {
			//		// make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
			//		// DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
			//
			//		// get reading from fradia
			//		// Send adequate command to cvFraDIA.
			//		memset(&to_vsp, 0, sizeof(lib::ECP_VSP_MSG));
			//		to_vsp_local = to_vsp;
			//		to_vsp_local.i_code = lib::VSP_GET_READING;
			//
			//		object_to_send_mutex.lock();
			//		memcpy(&to_vsp_local.fradia_sensor_command, &object_to_send_shared, sizeof(TO_VSP_T));
			//		memset(&object_to_send_shared, 0, sizeof(TO_VSP_T));
			//		object_to_send_mutex.unlock();
			//
			//		result = write(sockfd, &to_vsp_local, sizeof(lib::ECP_VSP_MSG));
			//		if (result < 0) {
			//			//TODO: exception
			//			logger::log("write() error.\n");
			//			continue;
			//			//throw sensor_error (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
			//		}
			//		if (result != sizeof(lib::ECP_VSP_MSG)) {
			//			logger::log("result != sizeof(lib::ECP_VSP_MSG)\n");
			//			continue;
			//		}
			//
			//		// Read aggregated data from cvFraDIA.
			//		result = read(sockfd, &from_vsp_local, sizeof(lib::VSP_ECP_MSG));
			//		if (result < 0) {
			//			//TODO: exception
			//			logger::log("read() error.\n");
			//			continue;
			//			//throw sensor_error (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
			//		}
			//		if (result != sizeof(lib::VSP_ECP_MSG)) {
			//			logger::log("result != sizeof(lib::VSP_ECP_MSG)\n");
			//			continue;
			//		}
			//
			//		if (from_vsp_local.vsp_report != lib::VSP_REPLY_OK) {
			//			logger::log("(from_vsp_local.vsp_report != lib::VSP_REPLY_OK\n");
			//			continue;
			//		}
			//
			//		get_reading_mutex.lock();
			//		// copy buffers
			//		memcpy(&received_object_shared, &from_vsp_local.comm_image.sensor_union.fradia_sensor_reading, sizeof(FROM_VSP_T));
			//
			//		get_reading_mutex.unlock();


			// send VSP_FRADIA_CONFIGURE_TASK (if object_to_send is set)
			bool configure_now = false;
			object_to_send_mutex.lock();
			configure_now = is_object_to_send_set;
			if (configure_now) {
				object_to_send = object_to_send_shared;
				is_object_to_send_set = false;
			}
			object_to_send_mutex.unlock();
			if (configure_now) {
				//logger::logDbg("fradia_sensor::operator() configure now 1\n");
				// send
				ecp_vsp_msg.i_code = lib::VSP_FRADIA_CONFIGURE_TASK;
				memcpy(ecp_vsp_msg.fradia_sensor_command, &object_to_send, sizeof(object_to_send));
				send_to_vsp(ecp_vsp_msg);
				//logger::logDbg("fradia_sensor::operator() configure now 2\n");
				// receive
				vsp_ecp_msg = receive_from_vsp();
				if (vsp_ecp_msg.vsp_report != lib::VSP_FRADIA_TASK_CONFIGURED) {
					logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): vsp_ecp_msg.vsp_report = %d != VSP_FRADIA_TASK_CONFIGURED\n", vsp_ecp_msg.vsp_report);
					throw std::logic_error("vsp_ecp_msg.vsp_report != lib::VSP_FRADIA_TASK_CONFIGURED");
				}
				memset(ecp_vsp_msg.fradia_sensor_command, 0, ECP_VSP_MSG_FRADIA_COMMAND_SIZE);
				//logger::logDbg("fradia_sensor::operator() configure now 3\n");
			}

			// send VSP_GET_READING
			ecp_vsp_msg.i_code = lib::VSP_GET_READING;
			send_to_vsp(ecp_vsp_msg);

			//logger::logDbg("fradia_sensor::operator() after get_reading\n");

			// get reading
			vsp_ecp_msg = receive_from_vsp();
			if (vsp_ecp_msg.vsp_report != lib::VSP_REPLY_OK) {
				logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): vsp_ecp_msg.vsp_report != lib::VSP_REPLY_OK\n");
				continue;
			}

			//logger::logDbg("fradia_sensor::operator() after receive\n");

			get_reading_mutex.lock();
			memcpy(&received_object_shared, &vsp_ecp_msg.comm_image.sensor_union.fradia_sensor_reading, sizeof(FROM_VSP_T));
			get_reading_mutex.unlock();

//			logger::logDbg("fradia_sensor::operator(): loop %d\n", ++iter);
		}
	} catch (const std::exception &ex) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): error %s\n", ex.what());
	} catch (...) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): unknown exception\n");
	}
	logger::logDbg("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()() end\n");
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
