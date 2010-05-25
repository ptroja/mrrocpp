/*!
 * \file ecp_mp_s_fradia_sensor.h
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
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp_mp/task/ecp_mp_task.h"

#include "lib/logger.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * \brief Types commands sent to PW_HaarDetect task.
 */
typedef enum _HD_MODE
{
	WITHOUT_ROTATION, PERFORM_ROTATION
} hd_mode_t;

/** Define size of data buffer for FraDIA <->MRROC++ communication. Used by CommunicationWrapper (FraDIA) and fradia_sensor (MRROC++) */
#define FRADIA_SENSOR_DATA_SIZE 24
#define FRADIA_SENSOR_PARAMETERS_SIZE 160
// char fradia_sensor_command[ECP_VSP_MSG_FRADIA_COMMAND_SIZE];

/*!
 * Class for communication with FraDIA. Parametrized by received structure.
 * make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
 * DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
 */
template <typename FROM_VSP_T, typename TO_VSP_T>
class fradia_sensor : public ecp_mp::sensor::sensor_interface
{
public:
	/**
	 * Message to the VSP
	 */
	typedef struct _ECP_VSP_MSG
	{
		lib::VSP_COMMAND_t i_code;
		union
		{
			TO_VSP_T parameters;
			unsigned char buffer[FRADIA_SENSOR_PARAMETERS_SIZE];
		};
	} ECP_VSP_MSG;

	/**
	 * Message from the VSP
	 */
	typedef struct _VSP_ECP_MSG
	{
		lib::VSP_REPORT_t vsp_report;
		union
		{
			FROM_VSP_T data;
			unsigned char buffer[FRADIA_SENSOR_DATA_SIZE];
		};
	} VSP_ECP_MSG;

	/**
	 * Create FraDIA sensor.
	 * Specified section must contain following options: fradia_node_name, fradia_port, fradia_task
	 * @param configurator
	 * @param section_name
	 */
	fradia_sensor(mrrocpp::lib::configurator& configurator, const std::string& section_name);

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

	/**
	 * Makes thread send object to fradia.
	 * @param object_to_send
	 */
	void configure_fradia_task(const TO_VSP_T& object_to_send);

	/*!
	 * MRROC++ <-> FraDIA communication thread method. Implements loop which queries FraDIA for new reading.
	 * make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
	 * DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
	 */
	void operator()();

	/*!
	 * Get report of the last data query
	 * @return status of the last communication with FraDIA
	 */
	lib::VSP_REPORT_t get_report(void) const;

	/** Object received from fradia, read by get_reading(). */
	FROM_VSP_T image;
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

	/*!
	 * Thread which continously read measurments from fradia over TCP/IP socket.
	 * Measurements are buffered and always available to get_reading().
	 */
	boost::shared_ptr <boost::thread> reader_thread;

	/**
	 * Mutex for received_object_shared.
	 */
	boost::mutex get_reading_mutex;

	/** This object holds data received from fradia.
	 * Object shared between threads.
	 */
	FROM_VSP_T received_object_shared;

	/** This object hold last report from FraDIA.
	 * Object shared between threads.
	 */
	lib::VSP_REPORT_t vsp_report_shared;

	/** This object hold report associated with the current image.
	 */
	lib::VSP_REPORT_t vsp_report;

	/** Set to true, when configure_fradia_task() has been called. */
	bool configure_fradia_task_now;
	/** This object will be send to configure fradia task.
	 * Object shared between threads.
	 */
	TO_VSP_T configure_fradia_task_shared;
	boost::mutex configure_fradia_task_mutex;

	/**
	 * Send object to fradia.
	 * @param ecp_vsp_msg
	 */
	void send_to_vsp(const ECP_VSP_MSG& ecp_vsp_msg);

	/**
	 * Receive object from fradia.
	 * @return
	 */
	VSP_ECP_MSG receive_from_vsp();

	/** Set to true by destructor to terminate thread. */
	bool terminate_now;
	boost::mutex terminate_now_mutex;
}; // class fradia_sensor

template <typename FROM_VSP_T, typename TO_VSP_T>
fradia_sensor <FROM_VSP_T, TO_VSP_T>::fradia_sensor(mrrocpp::lib::configurator& configurator, const std::string& section_name) :
	configurator(configurator), vsp_report_shared(lib::VSP_SENSOR_NOT_CONFIGURED),
			vsp_report(lib::VSP_SENSOR_NOT_CONFIGURED), configure_fradia_task_now(false), terminate_now(false)
{
	if (sizeof(FROM_VSP_T) > FRADIA_SENSOR_DATA_SIZE) {
		logger::log_dbg("sizeof(FROM_VSP_T): %d\n", (int) sizeof(FROM_VSP_T));
		logger::log_dbg("FRADIA_SENSOR_DATA_SIZE: %d\n", FRADIA_SENSOR_DATA_SIZE);
		throw std::logic_error("sizeof(FROM_VSP_T) > FRADIA_SENSOR_DATA_SIZE");
	}
	if (sizeof(TO_VSP_T) > FRADIA_SENSOR_PARAMETERS_SIZE) {
		logger::log_dbg("sizeof(TO_VSP_T): %d\n", (int) sizeof(TO_VSP_T));
		logger::log_dbg("FRADIA_SENSOR_PARAMETERS_SIZE: %d\n", FRADIA_SENSOR_PARAMETERS_SIZE);
		throw std::logic_error("sizeof(TO_VSP_T) > FRADIA_SENSOR_PARAMETERS_SIZE");
	}

	// Set period variables.
	base_period = current_period = 1;

	// Retrieve cvfradia node name and port from configuration file.
	int cvfradia_port = configurator.value <int> ("fradia_port", section_name);
	std::string cvfradia_node_name = configurator.value <std::string> ("fradia_node_name", section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		logger::log("ERROR opening socket");
		throw std::runtime_error("socket(): " + std::string(strerror(errno)));
	}

	// Get server hostname.
	server = gethostbyname(cvfradia_node_name.c_str());
	if (server == NULL) {
		logger::log("ERROR, no host '%s'\n", cvfradia_node_name.c_str());
		throw std::runtime_error("gethostbyname(): " + std::string(hstrerror(h_errno)));
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
		throw std::runtime_error("connect(): " + std::string(strerror(errno)));
	}

	// Retrieve task name.
	fradia_task = configurator.value <std::string> ("fradia_task", section_name);

	std::memset(&image, 0, sizeof(FROM_VSP_T));
	std::memset(&received_object_shared, 0, sizeof(FROM_VSP_T));
}

template <typename FROM_VSP_T, typename TO_VSP_T>
fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor()
{
	logger::log_dbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor() begin\n");

	{
		boost::interprocess::scoped_lock <boost::mutex> l(terminate_now_mutex);
		terminate_now = true;
	}
	//	ECP_VSP_MSG ecp_vsp_msg;
	//	ecp_vsp_msg.i_code = lib::VSP_TERMINATE;
	//	send_to_vsp(ecp_vsp_msg);

	if (reader_thread.get()) {
		reader_thread->join();
	}
	close(sockfd);
	logger::log_dbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor() end\n");
} // end: terminate()

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::send_to_vsp(const ECP_VSP_MSG& ecp_vsp_msg)
{
	int result;
	result = write(sockfd, &ecp_vsp_msg, sizeof(ECP_VSP_MSG));
	if (result < 0) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): write failed\n");
		throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
	if (result != sizeof(ECP_VSP_MSG)) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): result != sizeof(ECP_VSP_MSG)\n");
		throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
}

template <typename FROM_VSP_T, typename TO_VSP_T>
typename fradia_sensor <FROM_VSP_T, TO_VSP_T>::VSP_ECP_MSG fradia_sensor <FROM_VSP_T, TO_VSP_T>::receive_from_vsp()
{
	VSP_ECP_MSG vsp_ecp_msg;

	int result = read(sockfd, &vsp_ecp_msg, sizeof(VSP_ECP_MSG));
	if (result < 0) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): read failed\n");
		throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	}
	if (result != sizeof(VSP_ECP_MSG)) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): result != sizeof(VSP_ECP_MSG)\n");
		throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE);
	}
	return vsp_ecp_msg;
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor()
{
	ECP_VSP_MSG ecp_vsp_msg;
	VSP_ECP_MSG vsp_ecp_msg;

	// Send adequate command to cvFraDIA.
	ecp_vsp_msg.i_code = lib::VSP_CONFIGURE_SENSOR;
	// Name of required task is set in constructor.
	// TODO: check this!
	// strcpy(ecp_vsp_msg.cvfradia_task_name, fradia_task.c_str());

	send_to_vsp(ecp_vsp_msg);
	vsp_ecp_msg = receive_from_vsp();

	if (vsp_ecp_msg.vsp_report == lib::VSP_FRADIA_TASK_LOADED) {
		reader_thread = boost::shared_ptr <boost::thread>(new boost::thread(boost::bind(&fradia_sensor <FROM_VSP_T,
				TO_VSP_T>::operator(), this)));
	} else {
		//logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_sensor(): can not load FraDIA task: %s.\n", vsp_ecp_msg.comm_image.sensor_union.fradia_sensor_reading);
		throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}

	logger::log_dbg("fradia_sensor<FROM_VSP_T>::configure_sensor() end\n");
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::get_reading()
{
	boost::interprocess::scoped_lock <boost::mutex> l(get_reading_mutex);

	image = received_object_shared;
	vsp_report = vsp_report_shared;

	vsp_report_shared = lib::VSP_READING_NOT_READY;

	//logger::log_dbg("fradia_sensor::get_reading()\n");
}

template <typename FROM_VSP_T, typename TO_VSP_T>
lib::VSP_REPORT_t fradia_sensor <FROM_VSP_T, TO_VSP_T>::get_report(void) const
{
	return vsp_report;
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_fradia_task(const TO_VSP_T& object_to_send)
{
	boost::interprocess::scoped_lock <boost::mutex> l(configure_fradia_task_mutex);

	logger::log_dbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::configure_fradia_task()\n");
	configure_fradia_task_shared = object_to_send;

	configure_fradia_task_now = true;
}

template <typename FROM_VSP_T, typename TO_VSP_T>
void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()()
{
	ECP_VSP_MSG ecp_vsp_msg;
	VSP_ECP_MSG vsp_ecp_msg;

	logger::log_dbg("fradia_sensor::operator() begin\n");

	try {
		while (1) {
			// make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
			// DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
			// check termination condition
			{
				boost::interprocess::scoped_lock <boost::mutex> l(terminate_now_mutex);
				if (terminate_now) {
					break;
				}
			}
			// send VSP_FRADIA_CONFIGURE_TASK (if configure_fradia_task_shared is set)
			bool configure_now = false;
			{
				boost::interprocess::scoped_lock <boost::mutex> l(configure_fradia_task_mutex);
				configure_now = configure_fradia_task_now;
				if (configure_now) {
					ecp_vsp_msg.i_code = lib::VSP_FRADIA_CONFIGURE_TASK;
					ecp_vsp_msg.parameters = configure_fradia_task_shared;
					configure_fradia_task_now = false;
				}
			}

			if (configure_now) {
				send_to_vsp(ecp_vsp_msg);
				vsp_ecp_msg = receive_from_vsp();
				if (vsp_ecp_msg.vsp_report != lib::VSP_FRADIA_TASK_CONFIGURED) {
					logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): vsp_ecp_msg.vsp_report = %d != VSP_FRADIA_TASK_CONFIGURED\n", vsp_ecp_msg.vsp_report);
					throw std::runtime_error("vsp_ecp_msg.vsp_report != lib::VSP_FRADIA_TASK_CONFIGURED");
				}
			}

			// send VSP_GET_READING
			ecp_vsp_msg.i_code = lib::VSP_GET_READING;
			memset(ecp_vsp_msg.buffer, 0, FRADIA_SENSOR_PARAMETERS_SIZE);
			send_to_vsp(ecp_vsp_msg);

			//logger::log_dbg("fradia_sensor::operator() after get_reading\n");

			// get reading
			vsp_ecp_msg = receive_from_vsp();
			if (vsp_ecp_msg.vsp_report != lib::VSP_REPLY_OK) {
				logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): vsp_ecp_msg.vsp_report != lib::VSP_REPLY_OK\n");
				continue;
			}

			{
				boost::interprocess::scoped_lock <boost::mutex> l(get_reading_mutex);
				received_object_shared = vsp_ecp_msg.data;
				vsp_report_shared = vsp_ecp_msg.vsp_report;
			}
		}
	} catch (const std::exception &ex) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): error %s\n", ex.what());
	} catch (...) {
		logger::log("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()(): unknown exception\n");
	}
	logger::log_dbg("void fradia_sensor <FROM_VSP_T, TO_VSP_T>::operator()() end\n");
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
