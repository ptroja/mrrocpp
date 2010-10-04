/*!
 * @file
 * @brief Virtual sensor on the ECP/MP side used for communication with the FraDIA framework.
 *
 *
 * @author tkornuta
 * @author mboryn
 * @date 15.03.2008
 *
 * @ingroup SENSORS FRADIA_SENSOR
 */

#ifndef ECP_MP_S_FRADIA_SENSOR_H
#define ECP_MP_S_FRADIA_SENSOR_H

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>
#include <cstring>
#include <stdexcept>

#include "base/ecp_mp/ecp_mp_sensor.h"

#include "base/lib/logger.h"
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {


const lib::sensor::SENSOR_t SENSOR_FRADIA = "SENSOR_FRADIA";


/**
 * @brief Class for communication with FraDIA. Parametrized by received structure.
 *
 * To make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
 * DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
 *
 * @tparam CONFIGURE_T Structure sent from FraDIA along with the sensor configuration command.
 * @tparam READING_T Response structure - aggregated reading sent from FraDIA.
 * @tparam INITIATE_T Structure sent from FraDIA along with the reading initiation command, empty as default.
 *
 * @author mboryn
 * @author tkornuta
 *
 * @ingroup SENSORS,FRADIA_SENSOR
 */
template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T = lib::empty_t>
class fradia_sensor : public ecp_mp::sensor::sensor_interface
{
private:
	/** @brief Maximal size of task name. */
	const static int task_name_size = 256;

	/**
	 * @brief Message sent to FraDIA by configure_sensor()
	 */
	struct FRADIA_LOAD_TASK_COMMAND
	{
		/** @brief Name of task to load. */
		char task_name[task_name_size];

		/** Should be equal to sizeof(CONFIGURE_T) - filled automatically by fradia_sensor. */
		size_t configure_size;

		/** Additional parameters sent with the sensor configuration command. */
		CONFIGURE_T configure_message;
	};

	/** Task loading report received from FraDIA. */
	enum FRADIA_TASK_LOAD_STATUS
	{
		FRADIA_TASK_NOT_EXISTS, FRADIA_TASK_LOADED,
	};

	/**
	 * @brief Task loading report received from FraDIA.
	 */
	struct FRADIA_LOAD_TASK_REPLY
	{
		/** @brief Should be equal to sent task name. */
		char task_name[task_name_size];

		/** @brief Should be equal to sizeof(READING_T) - filled automatically by fradia_sensor. */
		size_t reading_size;

		/** @brief Should be equal to sizeof(INITIATE_T) - filled automatically by fradia_sensor. */
		size_t initiate_size;

		/** @brief Operation status. */
		FRADIA_TASK_LOAD_STATUS status;
	};

	/** @brief Task name to load. */
	const std::string fradia_task;

	/** @brief Socket file descriptor.  */
	int sockfd;

	/** @brief Configuration object. */
	mrrocpp::lib::configurator& configurator;

	/** @brief Report set by get_reading(). */
	lib::sensor::VSP_REPORT_t report;

	/** @brief Sensor configuration message. */
	CONFIGURE_T configure_message;

	/**  @brief Reading initialization message. */
	INITIATE_T initiate_message;

	/** @brief Retrieved reading. */
	READING_T reading_message;

	/** @brief Flag used to inform whether initiate message should be sent. False from default. */
	bool send_initiate_message;

	/**
	 * @brief Sends single message to FraDIA.
	 *
	 * Throws exception on error.
	 *
	 * @tparam MESSAGE_T Type of sent message.
	 * @param message Sent message.
	 */
	template <typename MESSAGE_T>
	void send_to_fradia(const MESSAGE_T& message);

	/**
	 * @brief Receives single message from FraDIA.
	 *
	 * Throws exception on error.
	 *
	 * @tparam MESSAGE_T Type of received message.
	 * @return Received message.
	 */
	template <typename MESSAGE_T>
	MESSAGE_T receive_from_fradia();

public:

	/**
	 * @brief FraDIA sensor constructor. Creates socket and connects to FraDIA, but doesn't send any data.
	 *
	 * @param _configurator Configuration object.
	 * @param section_name Name of the section in ini file. Specified section in config must contain following options: fradia_node_name, fradia_port, fradia_task.
	 * @param configure_message Message that will be send by configure_sensor().
	 */
	fradia_sensor(mrrocpp::lib::configurator& _configurator, const std::string& section_name, const CONFIGURE_T& configure_message = CONFIGURE_T());

	/**
	 * @brief Destructor - closes FraDIA socket connection.
	 */
	~fradia_sensor();

	/**
	 * @brief Sends configuration message (with task name and CONFIGURE_T  message). Then waits for reply.
	 * Exception is thrown if task has not been loaded.
	 */
	void configure_sensor();

	/**
	 * @brief Sends INITIATE_T command to FraDIA.
	 */
	void initiate_reading();

	/**
	 * @brief Receives READING_T message.
	 */
	void get_reading();

	/**
	 * @brief Get report of the last data query.
	 * @return status of the last communication with FraDIA
	 */
	lib::sensor::VSP_REPORT_t get_report(void) const;

	/**
	 * @brief Set message to be sent by initiate_reading().
	 * @param msg Message to be sent.
	 */
	void set_initiate_message(const INITIATE_T& msg);
	/**
	 * @brief Get message received by get_reading().
	 *
	 * Throws exception if sensor is not configured.
	 *
	 * @return Reading retrieved from FraDIA.
	 */
	const READING_T& get_reading_message() const;

}; // class fradia_sensor

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::fradia_sensor(mrrocpp::lib::configurator& _configurator, const std::string& section_name, const CONFIGURE_T& configure_message) :
	fradia_task(_configurator.value <std::string> ("fradia_task", section_name)),
	configurator(_configurator),
	report(lib::sensor::VSP_SENSOR_NOT_CONFIGURED),
	configure_message(configure_message),
	send_initiate_message(false)
{
	// Set period variables.
	base_period = current_period = 1;

	// Retrieve FraDIA node name and port from configuration file.
	uint16_t fradia_port = configurator.value <uint16_t> ("fradia_port", section_name);
	const std::string fradia_node_name = configurator.value <std::string> ("fradia_node_name", section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		throw std::runtime_error("socket(): " + std::string(strerror(errno)));
	}

	int flag = 1;
	if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) == -1) {
		throw std::runtime_error("setsockopt(): " + std::string(strerror(errno)));
	}

	// Get server hostname.
	hostent * server = gethostbyname(fradia_node_name.c_str());
	if (server == NULL) {
		throw std::runtime_error("gethostbyname(" + fradia_node_name + "): " + std::string(hstrerror(h_errno)));
	}

	// Data with address of connection
	sockaddr_in serv_addr;

	// Reset socketaddr data.
	memset(&serv_addr, 0, sizeof(serv_addr));

	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(fradia_port);

	// Try to establish a connection with FraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		throw std::runtime_error("connect(): " + std::string(strerror(errno)));
	}

	report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
	logger::log("FraDIA sensor created.\n");
	logger::log_dbg("fradia_sensor: sizeof(CONFIGURE_T) = %d\n", sizeof(CONFIGURE_T));
	logger::log_dbg("fradia_sensor: sizeof(READING_T) = %d\n", sizeof(READING_T));
	logger::log_dbg("fradia_sensor: sizeof(INITIATE_T) = %d\n", sizeof(INITIATE_T));
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::~fradia_sensor()
{
	close(sockfd);
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
template <typename MESSAGE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::send_to_fradia(const MESSAGE_T& message)
{
	//	logger::log_dbg("fradia_sensor::send_to_fradia(): sizeof(MESSAGE_T): %d\n", sizeof(MESSAGE_T));
	int result = write(sockfd, &message, sizeof(MESSAGE_T));
	if (result < 0) {
		throw std::runtime_error(std::string("write() failed: ") + strerror(errno));
	}
	if (result != sizeof(MESSAGE_T)) {
		throw std::runtime_error("write() failed: result != sizeof(MESSAGE_T)");
	}
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
template <typename MESSAGE_T>
MESSAGE_T fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::receive_from_fradia()
{
	//	logger::log_dbg("fradia_sensor::receive_from_fradia(): sizeof(MESSAGE_T): %d\n", sizeof(MESSAGE_T));
	MESSAGE_T message;

	int result = read(sockfd, &message, sizeof(MESSAGE_T));
	if (result < 0) {
		throw std::runtime_error(std::string("read() failed: ") + strerror(errno));
	}
	if (result != sizeof(MESSAGE_T)) {
		throw std::runtime_error("read() failed: result != sizeof(MESSAGE_T)");
	}

	//	logger::log_dbg("fradia_sensor::receive_from_fradia() end\n");

	return message;
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::configure_sensor()
{
	if (fradia_task.size() >= task_name_size) {
		throw std::runtime_error("fradia_task.size() >= task_name_max_length");
	}

	FRADIA_LOAD_TASK_COMMAND command;
	strcpy(command.task_name, fradia_task.c_str());
	command.configure_size = sizeof(CONFIGURE_T);
	command.configure_message = configure_message;
	send_to_fradia(command);

	//	logger::log_dbg("fradia_sensor::configure_sensor() 2\n");
	FRADIA_LOAD_TASK_REPLY status = receive_from_fradia <FRADIA_LOAD_TASK_REPLY> ();
	status.task_name[task_name_size - 1] = 0;
	if (fradia_task != status.task_name) {
		throw std::runtime_error("FraDIA reply not recognized");
	}
	if (status.initiate_size != sizeof(INITIATE_T)) {
		logger::log("sizeof(INITIATE_T): %d\n", sizeof(INITIATE_T));
		logger::log("status.initiate_size: %d\n", status.initiate_size);
		throw std::runtime_error("status.initiate_size != sizeof(INITIATE_T)");
	}
	if (status.reading_size != sizeof(READING_T)) {
		logger::log("sizeof(INITIATE_T): %d\n", sizeof(READING_T));
		logger::log("status.initiate_size: %d\n", status.reading_size);
		throw std::runtime_error("status.reading_size != sizeof(READING_T)");
	}
	if (status.status != FRADIA_TASK_LOADED) {
		throw std::runtime_error("Failed to load FraDIA task \"" + fradia_task + "\"");
	}

	// logger::log_dbg("fradia_sensor::configure_sensor() end\n");
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::initiate_reading()
{
	//logger::log_dbg("fradia_sensor::initiate_reading()\n");
	if (send_initiate_message) {
		fd_set wfds;

		FD_ZERO(&wfds);
		FD_SET(sockfd, &wfds);

		struct timeval tv;
		tv.tv_sec = tv.tv_usec = 0;

		int result = select(sockfd + 1, NULL, &wfds, NULL, &tv);
		if (result == -1) {
			throw std::runtime_error(std::string("select() failed: ") + strerror(errno));
		}
		if (result > 0) {
			logger::log("fradia_sensor::initiate_reading(): write will block.");
		}

		send_to_fradia(initiate_message);
		send_initiate_message = false;
	}
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_reading()
{
	//logger::log_dbg("fradia_sensor::get_reading()\n");
	if (report != lib::sensor::VSP_SENSOR_NOT_CONFIGURED) {
		report = lib::sensor::VSP_READING_NOT_READY;
	}

	while (1) {
		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(sockfd, &rfds);

		struct timeval tv;
		tv.tv_sec = tv.tv_usec = 0;

		int result = select(sockfd + 1, &rfds, NULL, NULL, &tv);
		//logger::log_dbg("fradia_sensor::get_reading(): select(): %d\n", result);
		if (result == -1) {
			throw std::runtime_error(std::string("select() failed: ") + strerror(errno));
		}
		if (result == 0) {
			break;
		}
//		logger::log_dbg("fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_reading() 1\n");
		reading_message = receive_from_fradia <READING_T> ();
//		logger::log_dbg("fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_reading() 2\n");
		report = lib::sensor::VSP_REPLY_OK;
	}
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
lib::sensor::VSP_REPORT_t fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_report(void) const
{
	return report;
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::set_initiate_message(const INITIATE_T& msg)
{
	//	logger::log_dbg("fradia_sensor::set_initiate_message()\n");
	initiate_message = msg;
	send_initiate_message = true;
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
const READING_T& fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_reading_message() const
{
	//logger::log_dbg("fradia_sensor::get_reading_message()\n");

	if (report == lib::sensor::VSP_SENSOR_NOT_CONFIGURED) {
		throw std::logic_error("fradia_sensor::get_reading_message(): report == lib::VSP_SENSOR_NOT_CONFIGURED");
	}

	return reading_message;
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
