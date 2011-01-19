/*
 * discode_sensor.cc
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <cstdio>
#include <sstream>
#include <sys/uio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "discode_sensor.h"
#include "base/lib/logger.h"
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
	state(DSS_NOT_CONNECTED), config(config), section_name(section_name), timer_print_enabled(false)
{
	base_period = current_period = 1;

	header_iarchive = shared_ptr <xdr_iarchive <> > (new xdr_iarchive <> );
	iarchive = shared_ptr <xdr_iarchive <> > (new xdr_iarchive <> );
	header_oarchive = shared_ptr <xdr_oarchive <> > (new xdr_oarchive <> );
	oarchive = shared_ptr <xdr_oarchive <> > (new xdr_oarchive <> );

	// determine size of rmh after serialization
	*header_oarchive << rmh;
	reading_message_header_size = header_oarchive->getArchiveSize();
	header_oarchive->clear_buffer();

	timer_init();
}

discode_sensor::~discode_sensor()
{

}

void discode_sensor::configure_sensor()
{
	timer_show("discode_sensor::configure_sensor() begin");

	if (state != DSS_NOT_CONNECTED) {
		state = DSS_ERROR;
		throw ds_wrong_state_exception("state != DSS_NOT_CONNECTED");
	}

	// Retrieve FraDIA node name and port from configuration file.
	uint16_t discode_port = config.value <uint16_t> ("discode_port", section_name);
	const string discode_node_name = config.value <string> ("discode_node_name", section_name);
	reading_timeout = config.value <double> ("discode_reading_timeout", section_name);
	rpc_call_timeout = config.value <double> ("discode_rpc_call_timeout", section_name);

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
		throw ds_connection_exception("gethostbyname(" + discode_node_name + "): " + string(hstrerror(h_errno)));
	}

	// Data with address of connection
	sockaddr_in serv_addr;

	// Reset socketaddr data.
	memset(&serv_addr, 0, sizeof(serv_addr));

	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(discode_port);

	// Try to establish a connection with FraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		state = DSS_ERROR;
		throw ds_connection_exception("connect(): " + string(strerror(errno)));
	}

	//	initiate_reading_object_set = false;
	rmh.data_size = 0;

	state = DSS_CONNECTED;

	log_dbg("discode_sensor::configure_sensor(): state = %d\n", (int)state);
	timer_show("discode_sensor::configure_sensor() end");
}

void discode_sensor::initiate_reading()
{
	timer_show("discode_sensor::initiate_reading() begin");
//	log_dbg("discode_sensor::initiate_reading(): state = %d\n", (int)state);
	if (!(state == DSS_CONNECTED || state == DSS_READING_RECEIVED || state == DSS_INITIATE_SENT)) {
		stringstream ss;
		ss
				<< "discode_sensor::initiate_reading(): !(state == DSS_CONNECTED || state == DSS_READING_RECEIVED || state == DSS_INITIATE_SENT): "
				<< state;
		state = DSS_ERROR;
		throw ds_wrong_state_exception(ss.str());
	}

	if (state == DSS_INITIATE_SENT) {
		check_reading_timeout();
		return;
	}

	//	if (!initiate_reading_object_set) {
	imh.data_size = 0;
	imh.is_rpc_call = false;
	oarchive->clear_buffer();
	//	}

	send_buffers_to_discode();

	//	initiate_reading_object_set = 0;
	state = DSS_INITIATE_SENT;
	if (clock_gettime(CLOCK_REALTIME, &initiate_sent_time) == -1) {
		throw runtime_error("clock_gettime() == -1");
	}
	timer_show("discode_sensor::initiate_reading() end");
}

void discode_sensor::check_reading_timeout()
{
	struct timespec current_time;
	if (clock_gettime(CLOCK_REALTIME, &current_time) == -1) {
		throw runtime_error("clock_gettime() == -1");
	}
	double delta_t = (current_time.tv_sec - initiate_sent_time.tv_sec) + (double) (current_time.tv_nsec
			- initiate_sent_time.tv_nsec) * 1e-9;
	if (delta_t > reading_timeout) {
		log_dbg("discode_sensor::check_reading_timeout(): state = %d\n", (int)state);
		state = DSS_ERROR;
		throw ds_timeout_exception("Timeout while waiting for reading.");
	}
}

void discode_sensor::get_reading()
{
	timer_show("discode_sensor::get_reading() begin");
	if (state != DSS_INITIATE_SENT) {
		state = DSS_ERROR;
		throw ds_wrong_state_exception("discode_sensor::get_reading(): state != DSS_INITIATE_SENT");
	}

	if (is_data_available()) {
		receive_buffers_from_discode();
		state = DSS_READING_RECEIVED;
	} else {
		check_reading_timeout();
	}

	timer_show("discode_sensor::get_reading() end");
}

void discode_sensor::terminate()
{
	if (!(state == DSS_CONNECTED || state == DSS_ERROR)) {
		throw ds_wrong_state_exception("discode_sensor::terminate(): !(state == DSS_CONNECTED || state == DSS_ERROR)");
	}
	close(sockfd);
	state = DSS_NOT_CONNECTED;
}

discode_sensor::discode_sensor_state discode_sensor::get_state()
{
//	log_dbg("discode_sensor::get_state(): state = %d\n", state);
	return state;
}

//bool discode_sensor::is_reading_ready()
//{
//	return rmh.data_size > 0 && rmh.is_rpc_call == false;
//}

bool discode_sensor::is_data_available(int usec)
{
	fd_set rfds;
	struct timeval tv;
	int retval;

	FD_ZERO(&rfds);
	FD_SET(sockfd, &rfds);
	tv.tv_sec = 0;
	tv.tv_usec = usec;

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

	header_iarchive->clear_buffer();
	int nread = read(sockfd, header_iarchive->get_buffer(), reading_message_header_size);
	if (nread == -1) {
		throw ds_connection_exception(string("read() failed: ") + strerror(errno));
	}
	if (nread != reading_message_header_size) {
		throw ds_connection_exception("read() failed: nread != reading_message_header_size");
	}

	*header_iarchive >> rmh;

	iarchive->clear_buffer();
	nread = read(sockfd, iarchive->get_buffer(), rmh.data_size);
	if (nread == -1) {
		throw ds_connection_exception(string("read() failed: ") + strerror(errno));
	}
	if (nread != rmh.data_size) {
		throw ds_connection_exception("read() failed: nread != rmh.data_size");
	}
	//	if (rmh.is_rpc_call) {
	//		throw runtime_error("void discode_sensor::receive_buffers_from_discode(): rmh.is_rpc_call");
	//	}
//	logger::log("discode_sensor::receive_buffers_from_discode() 3\n");

	if (rmh.data_size > 0) {
		state = DSS_READING_RECEIVED;
	} else {
		//		logger::log("discode_sensor::receive_buffers_from_discode() 2: no data available.\n");
		state = DSS_CONNECTED;
	}
}

void discode_sensor::send_buffers_to_discode()
{
	//	log_dbg("oarchive.getArchiveSize(): %d\n", oarchive->getArchiveSize());

	imh.data_size = oarchive->getArchiveSize();

	header_oarchive->clear_buffer();
	*header_oarchive << imh;

	struct iovec iov[2];
	ssize_t nwritten;

	iov[0].iov_base = (void*) header_oarchive->get_buffer();
	iov[0].iov_len = header_oarchive->getArchiveSize();
	iov[1].iov_base = (void*) oarchive->get_buffer();
	iov[1].iov_len = oarchive->getArchiveSize();

	nwritten = writev(sockfd, iov, 2);
	if (nwritten == -1) {
		throw ds_connection_exception("writev(sockfd, iov, 2) == -1");
	}
	if (nwritten != (int) (header_oarchive->getArchiveSize() + oarchive->getArchiveSize())) {
		throw ds_connection_exception("writev(sockfd, iov, 2) != header_oarchive->getArchiveSize() + oarchive->getArchiveSize()");
	}

	oarchive->clear_buffer();
}

void discode_sensor::timer_init()
{
	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::configure_sensor(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::timer_show(const char *str)
{
	if (timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED");
	}
	float seconds;
	if (timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED");
	}

	if (timer_print_enabled) {
		log_dbg("Time elapsed (in %s): %g s\n", str, seconds);
	}

	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

} // namespace discode

} // namespace mrrocpp

} // namespace ecp_mp

} // namespace mrrocpp
