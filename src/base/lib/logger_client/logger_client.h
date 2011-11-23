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
	logger_client(int buffer_size, const std::string& server_addr, int server_port);
	~logger_client();

	void log(log_message& msg);

	void operator()();
protected:

private:
	logger_client(const logger_client&);
	void connect();
	void send_message(const log_message& msg);
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

	xdr_oarchive<> oa_header;
	xdr_oarchive<> oa_data;
};

} // namespace logger

#endif /* LOGGER_CLIENT_H_ */
