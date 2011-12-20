/**
 * \file logger.h
 * \brief Logging utilities.
 * \bug Not multi-thread safe
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#ifndef LOGGER_CLIENT_H_
#define LOGGER_CLIENT_H_

#include <ctime>
#include <deque>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>

#include "base/lib/xdr/xdr_oarchive.hpp"
#include "log_message.h"

#include "base/lib/mrmath/homog_matrix.h"	// TODO: remove

namespace logger {

double time_diff(struct timespec t1, struct timespec t0);

class logger_client {
public:
	logger_client(int buffer_size, const std::string& server_addr, int server_port, const std::string& header_text = "", const std::string& filename_prefix = "");
	~logger_client();

	void log(log_message& msg);

	void operator()();

	void set_filename_prefix(const std::string& filename_prefix);
	void set_connect();
	void set_disconnect();
protected:

private:
	logger_client(const logger_client&);
	void connect();
	template<typename T>
	void send_message(const T& msg);
	void disconnect();

	int fd;
	boost::thread thread;
	boost::circular_buffer<log_message> buffer;

	std::string server_addr;
	int server_port;

	uint32_t current_message_number;
	boost::condition_variable cond;
	boost::mutex queue_mutex;
	bool terminate;
	bool connect_now;
	bool disconnect_now;
	bool connected;

	boost::condition_variable connect_cond;
	boost::condition_variable disconnect_cond;

	xdr_oarchive<> oa_header;
	xdr_oarchive<> oa_data;

	const std::string header_text;
	std::string filename_prefix;
};

template<typename T>
void logger_client::send_message(const T& msg)
{
	if(fd < 0){
		return;
	}
	oa_data.clear_buffer();
	oa_header.clear_buffer();

	oa_data << msg;

	log_message_header header;
	header.message_size = oa_data.getArchiveSize();
	oa_header << header;

	struct iovec iov[2];
	ssize_t nwritten;

	iov[0].iov_base = (void*) oa_header.get_buffer();
	iov[0].iov_len = oa_header.getArchiveSize();
	iov[1].iov_base = (void*) oa_data.get_buffer();
	iov[1].iov_len = oa_data.getArchiveSize();

	nwritten = writev(fd, iov, 2);
	if (nwritten == -1) {
		throw std::runtime_error("Socket::writev2() nwritten == -1");
	}
	if ((size_t) nwritten != iov[0].iov_len + iov[1].iov_len) {
		throw std::runtime_error("Socket::writev2() nwritten != buf1Size + buf2Size");
	}
}

} // namespace logger

#endif /* LOGGER_CLIENT_H_ */
