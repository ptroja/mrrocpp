/*
 * discode_sensor.cc
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <cstdio>
#include <sstream>
#include <cmath>

#include <sys/uio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "discode_sensor.h"
#include "base/lib/configurator.h"
#include "headers.h"

namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

namespace discode {

using namespace std;
using namespace logger;
using namespace boost;
using lib::configurator;

ds_exception::ds_exception(const string& arg) :
	runtime_error(arg)
{

}

ds_connection_exception::ds_connection_exception(const string& arg) :
	ds_exception("discode_sensor connection problem: " + arg)
{

}

ds_timeout_exception::ds_timeout_exception(const string& arg) :
	ds_exception("discode_sensor timeout: " + arg)
{

}

ds_wrong_state_exception::ds_wrong_state_exception(const string& arg) :
	ds_exception("discode_sensor wrong state: " + arg)
{

}

discode_sensor::discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name) :
	state(DSS_ERROR)
{
	base_period = current_period = 1;

	// read sensor settings from config.
	discode_port = config.value<uint16_t> ("discode_port", section_name);
	discode_node_name = config.value<string> ("discode_node_name", section_name);
	reading_timeout = config.value<double> ("discode_reading_timeout", section_name);
	rpc_call_timeout = config.value<double> ("discode_rpc_call_timeout", section_name);

	// determine size of rmh after serialization
	header_oarchive << rmh;
	reading_message_header_size = header_oarchive.getArchiveSize();
	header_oarchive.clear_buffer();

	state = DSS_NOT_CONNECTED;
}

discode_sensor::~discode_sensor()
{

}

void discode_sensor::configure_sensor()
{
	if (state != DSS_NOT_CONNECTED) {
		state = DSS_ERROR;
		throw ds_wrong_state_exception("state != DSS_NOT_CONNECTED");
	}

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		state = DSS_ERROR;
		throw ds_connection_exception("socket(): " + string(strerror(errno)));
	}

	int flag = 1;
	if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) == -1) {
		state = DSS_ERROR;
		throw ds_connection_exception("setsockopt(): " + string(strerror(errno)));
	}

	// Get server hostname.
	hostent * server = gethostbyname(discode_node_name.c_str());
	if (server == NULL) {
		state = DSS_ERROR;
		throw ds_connection_exception("discode_sensor::configure_sensor(): gethostbyname(" + discode_node_name + "): " + string(
				hstrerror(h_errno)));
	}

	// Data with address of connection
	sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(discode_port);

	// Try to establish a connection with discode.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		state = DSS_ERROR;
		throw ds_connection_exception("connect(): " + string(strerror(errno)));
	}

	// there was no reading
	rmh.data_size = 0;

	// connected to discode
	state = DSS_CONNECTED;
}

void discode_sensor::initiate_reading()
{
}

void discode_sensor::get_reading()
{
	if (state == DSS_REQUEST_SENT) {
		// in last call to get_reading() request was sent, but no reading was received,
		// so try receiving reading now.
		if (is_data_available()) {
			// reading has been received.
			receive_buffers_from_discode();
			save_reading_received_time();
			state = rmh.data_size > 0 ? DSS_READING_RECEIVED : DSS_CONNECTED;
		} else {
			// no data received - too long for receiving reading from Discode.
			// there must be something wrong with connection to discode.
			throw ds_timeout_exception(
					"discode_sensor: Timeout while waiting for reading. Check connection to DisCODe and task running on DisCODe.");
		}
	} else if (state == DSS_CONNECTED || state == DSS_READING_RECEIVED) {
		// send request to DisCODe.
		imh.data_size = 0;
		imh.is_rpc_call = false;
		oarchive.clear_buffer();
		save_request_sent_time();
		send_buffers_to_discode();
		if (is_data_available(reading_timeout)) {
			// reading has just been received
			receive_buffers_from_discode();
			save_reading_received_time();
			state = rmh.data_size > 0 ? DSS_READING_RECEIVED : DSS_CONNECTED;
		} else {
			// timeout, try receiving in next call to get_reading()
			log_dbg("discode_sensor::get_reading(): timeout, state = DSS_REQUEST_SENT\n");
			state = DSS_REQUEST_SENT;
		}
	} else {
		stringstream ss;
		ss
				<< "discode_sensor::get_reading(): !(state == DSS_CONNECTED || state == DSS_READING_RECEIVED || state == DSS_REQUEST_SENT): "
				<< state;
		state = DSS_ERROR;
		throw ds_wrong_state_exception(ss.str());
	}
}

void discode_sensor::terminate()
{
	if (!(state == DSS_CONNECTED || state == DSS_ERROR)) {
		throw ds_wrong_state_exception(
				"discode_sensor::terminate(): !(state == DSS_CONNECTED || state == DSS_ERROR)");
	}
	close(sockfd);
	state = DSS_NOT_CONNECTED;
}

discode_sensor::discode_sensor_state discode_sensor::get_state()
{
	//	log_dbg("discode_sensor::get_state(): state = %d\n", state);
	return state;
}

bool discode_sensor::is_data_available(double sec)
{
	fd_set rfds;
	struct timeval tv;
	int retval;

	FD_ZERO(&rfds);
	FD_SET(sockfd, &rfds);
	tv.tv_sec = floor(sec);
	tv.tv_usec = round(fmod(sec, 1) * 1e6);

	//	log_dbg("discode_sensor::is_data_available(%g): tv.tv_sec=%ld; tv.tv_usec=%ld\n", sec, (long int)tv.tv_sec, (long int)tv.tv_usec);

	retval = select(sockfd + 1, &rfds, NULL, NULL, &tv);

	if (retval < 0) {
		throw ds_connection_exception("discode_sensor::is_data_available(): select() failed: "
				+ string(strerror(errno)));
	}
	return retval > 0;
}

void discode_sensor::receive_buffers_from_discode()
{
	//	logger::log("discode_sensor::receive_buffers_from_discode() 1\n");

	header_iarchive.clear_buffer();
	int nread = read(sockfd, header_iarchive.get_buffer(), reading_message_header_size);
	if (nread == -1) {
		throw ds_connection_exception(string("read() failed: ") + strerror(errno));
	}
	if (nread != reading_message_header_size) {
		throw ds_connection_exception("read() failed: nread != reading_message_header_size");
	}

	header_iarchive >> rmh;

	iarchive.clear_buffer();
	nread = read(sockfd, iarchive.get_buffer(), rmh.data_size);
	if (nread == -1) {
		throw ds_connection_exception(string("read() failed: ") + strerror(errno));
	}
	if (nread != rmh.data_size) {
		char txt[128];
		sprintf(txt, "read() failed: nread(%d) != rmh.data_size(%d)", nread, rmh.data_size);
		throw ds_connection_exception(txt);
	}
}

void discode_sensor::send_buffers_to_discode()
{
	imh.data_size = oarchive.getArchiveSize();

	header_oarchive.clear_buffer();
	header_oarchive << imh;

	struct iovec iov[2];
	ssize_t nwritten;

	iov[0].iov_base = (void*) header_oarchive.get_buffer();
	iov[0].iov_len = header_oarchive.getArchiveSize();
	iov[1].iov_base = (void*) oarchive.get_buffer();
	iov[1].iov_len = oarchive.getArchiveSize();

	nwritten = writev(sockfd, iov, 2);
	if (nwritten == -1) {
		throw ds_connection_exception("writev(sockfd, iov, 2) == -1");
	}
	if (nwritten != (int) (header_oarchive.getArchiveSize() + oarchive.getArchiveSize())) {
		throw ds_connection_exception(
				"writev(sockfd, iov, 2) != header_oarchive.getArchiveSize() + oarchive.getArchiveSize()");
	}

	oarchive.clear_buffer();
}

reading_message_header discode_sensor::get_rmh() const
{
	if (state != DSS_READING_RECEIVED) {
		state = DSS_ERROR;
		throw ds_wrong_state_exception(
				"discode_sensor::get_rmh(): state != DSS_READING_RECEIVED");
	}
	return rmh;
}

struct timespec discode_sensor::get_request_sent_time() const
{
	return request_sent_time;
}

struct timespec discode_sensor::get_reading_received_time() const
{
	return reading_received_time;
}

void discode_sensor::save_request_sent_time()
{
	struct timespec ts;
	if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
		request_sent_time = ts;
	} else {
		request_sent_time.tv_sec = 0;
		request_sent_time.tv_nsec = 0;
	}
}
void discode_sensor::save_reading_received_time()
{
	struct timespec ts;
	if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
		reading_received_time = ts;
	} else {
		reading_received_time.tv_sec = 0;
		reading_received_time.tv_nsec = 0;
	}
}

double discode_sensor::get_mrroc_discode_time_offset() const
{
	int seconds, nanoseconds;
	seconds = (reading_received_time.tv_sec + request_sent_time.tv_sec) / 2 - rmh.sendTimeSeconds;
	nanoseconds = (reading_received_time.tv_nsec + request_sent_time.tv_nsec) / 2
			- rmh.sendTimeNanoseconds;
	double offset = seconds + 1e-9 * nanoseconds;

	seconds = reading_received_time.tv_sec - request_sent_time.tv_sec;
	nanoseconds = reading_received_time.tv_nsec - request_sent_time.tv_nsec;
	double comm_time = (seconds + 1e-9 * nanoseconds);

	if (comm_time > 0.01) { // check if there was communication timeout (there was one macrostep between sending request and receiving reply)
		log_dbg("discode_sensor::get_mrroc_discode_time_offset(): comm_time > 0.01\n");
	}

	return offset;
}

} // namespace discode

} // namespace mrrocpp

} // namespace ecp_mp

} // namespace mrrocpp
