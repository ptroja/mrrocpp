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

#include <boost/shared_ptr.hpp>

#include "ecp_mp/sensor/ecp_mp_sensor.h"

#include "lib/logger.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * Class for communication with FraDIA. Parametrized by received structure.
 * make fradia process blocking queries: in FraDIA ImageProcessor's constructor make sure you call
 * DataSynchronizer::getInstance()->setWaitForVSPResponse(true);
 */
template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T = lib::empty_t>
class fradia_sensor : public ecp_mp::sensor::sensor_interface
{
private:
	struct FRADIA_COMMAND_HEADER
	{
		lib::VSP_COMMAND_t i_code;
		size_t data_size;
	};

	struct FRADIA_REPLY_HEADER
	{
		lib::VSP_REPORT_t vsp_report;
		size_t data_size;
	};

	const static int task_name_max_length = 256;

	struct FRADIA_COMMAND_LOAD_TASK
	{
		char task_name[task_name_max_length];
	};

public:

	/**
	 * Create FraDIA sensor.
	 * Specified section must contain following options: fradia_node_name, fradia_port, fradia_task
	 * @param configurator
	 * @param section_name
	 * @param configure_message will be send by configure_sensor()
	 */
			fradia_sensor(mrrocpp::lib::configurator& configurator, const std::string& section_name, const CONFIGURE_T& configure_message);

	/*!
	 * Closes cvFraDIA socket connection.
	 */
	~fradia_sensor();

	/**
	 * Loads FraDIA task and sends configuration message. Then waits for reply.
	 * Exception is thrown if task has not been loaded.
	 */
	void configure_sensor();

	/**
	 * Sends INITIATE_T
	 */
	void initiate_reading();

	/**
	 * Receives READING_T
	 */
	void get_reading();

	/**
	 * Get report of the last data query
	 * @return status of the last communication with FraDIA
	 */
	lib::VSP_REPORT_t get_report(void) const;

	void set_initiate_message(const INITIATE_T& msg);
	const READING_T& get_reading_message() const;
private:
	std::string fradia_task;

	/*!
	 * Socket file descriptor.
	 */
	int sockfd;

	/** Configurator. */
	mrrocpp::lib::configurator& configurator;

	CONFIGURE_T configure_message;

	bool send_initiate_message;
	INITIATE_T initiate_message;

	FRADIA_REPLY_HEADER reply_header;
	READING_T reading_message;

	template <typename MESSAGE_T>
	void send_to_fradia(const MESSAGE_T& message);

	template <typename MESSAGE_T>
	MESSAGE_T receive_from_fradia();

	bool reading_not_initialized;
}; // class fradia_sensor

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::fradia_sensor(mrrocpp::lib::configurator& configurator, const std::string& section_name, const CONFIGURE_T& configure_message) :
	configurator(configurator), configure_message(configure_message), send_initiate_message(false),
			reading_not_initialized(true)
{
	sockaddr_in serv_addr;
	hostent* server;

	// Set period variables.
	base_period = current_period = 1;

	// Retrieve cvfradia node name and port from configuration file.
	int cvfradia_port = configurator.value <int> ("fradia_port", section_name);
	std::string cvfradia_node_name = configurator.value <std::string> ("fradia_node_name", section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		throw std::runtime_error("socket(): " + std::string(strerror(errno)));
	}

	// Get server hostname.
	server = gethostbyname(cvfradia_node_name.c_str());
	if (server == NULL) {
		throw std::runtime_error("gethostbyname(" + cvfradia_node_name + "): " + std::string(hstrerror(h_errno)));
	}

	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *) server->h_addr, (char *) &serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(cvfradia_port);

	// Try to establish a connection with cvFraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		throw std::runtime_error("connect(): " + std::string(strerror(errno)));
	}

	// Retrieve task name.
	fradia_task = configurator.value <std::string> ("fradia_task", section_name);

	reply_header.vsp_report = lib::VSP_SENSOR_NOT_CONFIGURED;
	reply_header.data_size = 0;

	logger::log("Communication channel to FraDIA created.\n");
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::~fradia_sensor()
{

	close(sockfd);
	logger::log_dbg("fradia_sensor <FROM_VSP_T, TO_VSP_T>::~fradia_sensor() end\n");
} // end: terminate()

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
//	logger::log_dbg("fradia_sensor::configure_sensor() 1\n");
	FRADIA_COMMAND_HEADER header;
	header.i_code = lib::VSP_CONFIGURE_SENSOR;
	header.data_size = sizeof(FRADIA_COMMAND_LOAD_TASK) + sizeof(CONFIGURE_T);
	send_to_fradia(header);

	FRADIA_COMMAND_LOAD_TASK load_task_command;
	if (fradia_task.size() >= task_name_max_length) {
		throw std::runtime_error("fradia_task.size() >= task_name_max_length");
	}
	strcpy(load_task_command.task_name, fradia_task.c_str());
	send_to_fradia(load_task_command);
	send_to_fradia(configure_message);

//	logger::log_dbg("fradia_sensor::configure_sensor() 2\n");

	while (1) {
		reply_header = receive_from_fradia <FRADIA_REPLY_HEADER> ();
		if (reply_header.vsp_report != lib::VSP_REPLY_OK) {
			throw std::runtime_error("error loading FraDIA task");
		}
		if (reply_header.data_size == 0) {
			break;
		}
	}
//	logger::log_dbg("fradia_sensor::configure_sensor() end\n");
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::initiate_reading()
{
//	logger::log_dbg("fradia_sensor::initiate_reading()\n");
	if (send_initiate_message) {
		FRADIA_COMMAND_HEADER header;
		header.i_code = lib::VSP_INITIATE_READING;
		header.data_size = sizeof(initiate_message);

		// TODO: select() komunikat o blokowaniu

		send_to_fradia(header);
		send_to_fradia(initiate_message);

		send_initiate_message = false;
	}
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
void fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_reading()
{
//	logger::log_dbg("fradia_sensor::get_reading()\n");
	reply_header.vsp_report = lib::VSP_READING_NOT_READY;

	while (1) {
		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(sockfd, &rfds);

		struct timeval tv;
		tv.tv_sec = tv.tv_usec = 0;

		int result = select(sockfd + 1, &rfds, NULL, NULL, &tv);
//		logger::log_dbg("fradia_sensor::get_reading(): select(): %d\n", result);
		if (result == -1) {
			throw std::runtime_error(std::string("select() failed: ") + strerror(errno));
		}
		if (result == 0) {
			break;
		}
		reply_header = receive_from_fradia <FRADIA_REPLY_HEADER> ();
		if (reply_header.data_size != sizeof(READING_T)) {
			char txt[16];
			sprintf(txt, "%d", reply_header.data_size);
			throw std::runtime_error("reply_header.data_size != sizeof(READING_T): reply_header.data_size="
					+ std::string(txt));
		}
		reading_message = receive_from_fradia <READING_T> ();

		reading_not_initialized = false;
	}
}

template <typename CONFIGURE_T, typename READING_T, typename INITIATE_T>
lib::VSP_REPORT_t fradia_sensor <CONFIGURE_T, READING_T, INITIATE_T>::get_report(void) const
{
//	logger::log_dbg("fradia_sensor::get_report()\n");

	if (reading_not_initialized) {
		return lib::VSP_SENSOR_NOT_CONFIGURED;
	}
	return reply_header.vsp_report;
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
//	logger::log_dbg("fradia_sensor::get_reading_message()\n");

	if (reading_not_initialized) {
		throw std::logic_error("fradia_sensor::get_reading_message(): reading_not_initialized");
	}

	return reading_message;
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
