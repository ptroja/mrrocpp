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

#include "base/ecp_mp/ecp_mp_sensor.h"
#include "base/lib/configurator.h"
#include "base/lib/timer.h"
#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"
#include "base/lib/logger.h"
#include "headers.h"


namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

namespace discode {

class discode_sensor : public mrrocpp::ecp_mp::sensor::sensor_interface
{
public:
//	enum discode_sensor_state{
//		NOT_CONFIGURED, INITIATE_SET, INITIATE_SENT, REA
//	};


	discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name);
	virtual ~discode_sensor();

	/**
	 * Receives data from discode.
	 */
	virtual void get_reading();

	/**
	 * Virtual method responsible for sensor configuration.
	 */
	virtual void configure_sensor();

	/**
	 * Virtual method responsible for reading initialization.
	 */
	virtual void initiate_reading();

	/**
	 * @brief Disconnect from discode.
	 */
	virtual void terminate();

	/**
	 * @brief Set object to be send by initiate_reading().
	 * @param initiate_object this object will be serialized to buffer and send by initiate_reading().
	 */
//	template <typename INITIATE_T>
//	void set_initiate_object(const INITIATE_T& initiate_object);

	/**
	 *
	 * @return True, if get_reading() received data from discode. False otherwise.
	 */
	bool is_reading_ready();

	/**
	 * @brief Get object received by get_reading().
	 * This method may be called only once after get_reading(). Just because.
	 * @return
	 */
	template <typename READING_T>
	READING_T get_received_object();

	/**
	 * @brief Sends data immediately and waits for response.
	 * @param to_send Data to send.
	 * @return data returned from call.
	 */
	template <typename RECEIVED_T, typename TO_SEND_T>
	RECEIVED_T call_remote_procedure(const TO_SEND_T& to_send);
private:
	mrrocpp::lib::configurator& config;
	const std::string section_name;

	boost::shared_ptr <xdr_iarchive <> > header_iarchive;
	boost::shared_ptr <xdr_iarchive <> > iarchive;
	boost::shared_ptr <xdr_oarchive <> > header_oarchive;
	boost::shared_ptr <xdr_oarchive <> > oarchive;

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

	bool initiate_reading_object_set;

	/**
	 * @brief Returns true, if there's data available to read.
	 * @param usec Timeout, if 0 is passed, method returns immediately.
	 * @return
	 */
	bool is_data_available(int usec);

	/**
	 * @brief Receives data from discode and puts it to header_iarchive and iarchive.
	 */
	void receive_buffers_from_discode();

	/**
	 * @brief Fills header_oarchive, then sends header_oarchive and oarchive to discode.
	 */
	void send_buffers_to_discode();

	// timer stuff, TODO: remove after discode_sensor is considered bug-free.
	mrrocpp::lib::timer timer;
	bool timer_print_enabled;
	void timer_init();
	void timer_show(const char *str = "");
}; // class discode_sensor

//template <typename INITIATE_T>
//void discode_sensor::set_initiate_object(const INITIATE_T& initiate_object)
//{
//	imh.is_rpc_call = false;
//
//	oarchive->clear_buffer();
//	*oarchive << initiate_object;
//}

template <typename READING_T>
READING_T discode_sensor::get_received_object()
{
	// TODO: check if get_reading has been called and data has been read.
	READING_T reading;

	*iarchive >> reading;

	return reading;
}

template <typename RECEIVED_T, typename TO_SEND_T>
RECEIVED_T discode_sensor::call_remote_procedure(const TO_SEND_T& to_send)
{
	logger::log("discode_sensor::call_remote_procedure() begin\n");
	imh.is_rpc_call = true;
	oarchive->clear_buffer();
	*oarchive << to_send;

	logger::log("discode_sensor::call_remote_procedure() before send_buffers\n");
	send_buffers_to_discode();

	logger::log("discode_sensor::call_remote_procedure() before loop\n");
	do{
		logger::log("discode_sensor::call_remote_procedure() inside loop\n");
		receive_buffers_from_discode();
	}while(!rmh.is_rpc_call); // skip non-RPC messages

	RECEIVED_T received;

	*iarchive >> received;

	initiate_reading_object_set = false;

	logger::log("discode_sensor::call_remote_procedure() end\n");

	return received;
}

} // namespace discode

} // namespace sensor

} // namespace ecp_mp

} // namespace mrrocpp

#endif /* DISCODE_SENSOR_H_ */
