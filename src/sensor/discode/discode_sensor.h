/*
 * discode_sensor.h
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#ifndef DISCODE_SENSOR_H_
#define DISCODE_SENSOR_H_

#include <string>
#include <cstring>
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <time.h>

#include "base/ecp_mp/ecp_mp_sensor.h"
#include "base/lib/configurator.h"
#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"
#include "base/lib/logger.h"
#include "headers.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {
namespace discode {

class ds_exception : public std::runtime_error
{
public:
	/** Takes a character string describing the error.  */
	explicit ds_exception(const std::string& arg);
};

class ds_connection_exception : public ds_exception
{
public:
	explicit ds_connection_exception(const std::string& arg);
};

class ds_timeout_exception : public ds_exception
{
public:
	explicit ds_timeout_exception(const std::string& arg);
};

class ds_wrong_state_exception : public ds_exception
{
public:
	explicit ds_wrong_state_exception(const std::string& arg);
};

class discode_sensor : public mrrocpp::ecp_mp::sensor::sensor_interface
{
public:
	/**
	 * DSS (short form of discode_sensor_state). State of the sensor.
	 */
	enum discode_sensor_state
	{
		DSS_NOT_CONNECTED, DSS_CONNECTED, DSS_REQUEST_SENT, DSS_READING_RECEIVED, DSS_ERROR
	};

	discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name);
	virtual ~discode_sensor();

	/**
	 * Virtual method responsible for sensor configuration.
	 */
	virtual void configure_sensor();

	/**
	 * Virtual method responsible for reading initialization.
	 */
	virtual void initiate_reading();

	/**
	 * Receives data from discode.
	 */
	virtual void get_reading();

	/**
	 * @brief Disconnect from discode.
	 */
	virtual void terminate();

	/**
	 * Get current state of discode_sensor.
	 * @return
	 */
	discode_sensor_state get_state();

	/**
	 * @brief Get object received by get_reading().
	 * This method may be called only once after get_reading(). Just because.
	 * @return
	 */
	template <typename READING_T>
	READING_T retreive_reading();

	/**
	 * @brief Sends data immediately and waits for response.
	 * @param to_send Data to send.
	 * @return data returned from call.
	 */
	template <typename RECEIVED_T, typename TO_SEND_T>
	RECEIVED_T call_remote_procedure(const TO_SEND_T& to_send);

	reading_message_header get_rmh() const;
	struct timespec get_reading_received_time() const;
	struct timespec get_request_sent_time() const;
	double get_mrroc_discode_time_offset() const;
private:
	mutable discode_sensor_state state;
	uint16_t discode_port;
	std::string discode_node_name;

	xdr_iarchive <> header_iarchive;
	xdr_iarchive <> iarchive;
	xdr_oarchive <> header_oarchive;
	xdr_oarchive <> oarchive;

	/** @brief Socket file descriptor.  */
	int sockfd;

	/** Size of reading_message_header in XDR */
	int reading_message_header_size;

	/**
	 * @brief Header of data to be received.
	 * Modified by receive_buffers_from_discode().
	 */
	reading_message_header rmh;

	/**
	 * @brief Header of data to be sent by initiate_reading().
	 * Modified by send_buffers_to_discode().
	 */
	initiate_message_header imh;

	/**
	 * @brief Returns true, if there's data available to read.
	 * @param sec Timeout, if 0 is passed, method returns without blocking.
	 * @return
	 */
	bool is_data_available(double sec = 0);

	/**
	 * @brief Receives data from discode and puts it to header_iarchive and iarchive.
	 */
	void receive_buffers_from_discode();

	/**
	 * @brief Fills header_oarchive, then sends header_oarchive and oarchive to discode.
	 */
	void send_buffers_to_discode();

	double reading_timeout;
	double rpc_call_timeout;
	struct timespec request_sent_time;
	struct timespec reading_received_time;

	void save_request_sent_time();
	void save_reading_received_time();
};
// class discode_sensor

template <typename READING_T>
READING_T discode_sensor::retreive_reading()
{
	if (state != DSS_READING_RECEIVED) {
		state = DSS_ERROR;
		throw ds_wrong_state_exception("discode_sensor::retreive_reading(): state != DSS_READING_RECEIVED");
	}

	READING_T reading;

	//	logger::log_dbg("discode_sensor::retreive_reading(): iarchive->getArchiveSize()=%zd\n", iarchive->getArchiveSize());

	iarchive >> reading;

	state = DSS_CONNECTED;
	return reading;
}

template <typename RECEIVED_T, typename TO_SEND_T>
RECEIVED_T discode_sensor::call_remote_procedure(const TO_SEND_T& to_send)
{
//	logger::log("discode_sensor::call_remote_procedure() begin\n");

	if (state != DSS_CONNECTED && state != DSS_READING_RECEIVED) {
		throw ds_wrong_state_exception("discode_sensor::call_remote_procedure(): state != DSS_CONNECTED");
	}

	imh.is_rpc_call = true;
	oarchive.clear_buffer();
	oarchive << to_send;

//	logger::log("discode_sensor::call_remote_procedure() before send_buffers\n");
	send_buffers_to_discode();

	if (is_data_available(rpc_call_timeout)) {
		receive_buffers_from_discode();
		RECEIVED_T received;

		for (int i = 0; i < rmh.data_size; ++i) {
			unsigned char c = iarchive.get_buffer()[i];
			logger::log("%02X ", (unsigned int) c);
		}
		logger::log("\n");

		logger::log("discode_sensor::call_remote_procedure() rmh.data_size: %d\n", rmh.data_size);
		logger::log("discode_sensor::call_remote_procedure() iarchive.size: %d\n", (int) iarchive.getArchiveSize());
		iarchive >> received;
		if (!rmh.is_rpc_call) {
			state = DSS_ERROR;
			throw ds_connection_exception("Received non-RPC reply to RPC call.");
		}

		state = DSS_CONNECTED;

		return received;
	} else {
		state = DSS_ERROR;
		throw ds_timeout_exception("Timeout while waiting for RPC result from DisCODe.");
	}

	//logger::log("discode_sensor::call_remote_procedure() end\n");
}

} // namespace discode
} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* DISCODE_SENSOR_H_ */
