	/**
 * \file logger.cc
 * \brief Logging utilities.
 * \bug Not multi-thread safe use of log_*_enabled flags
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#include <cstdio>
#include <cstdarg>

#include <sys/uio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "base/lib/logger_client/logger_client.h"

using namespace std;

namespace logger {


double time_diff(struct timespec t1, struct timespec t0)
{
	return (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
}

logger_client::logger_client(int buffer_size, const std::string& server_addr, int server_port) :
		fd(-1), buffer(buffer_size), server_addr(server_addr), server_port(server_port), current_message_number(0), terminate(false)
{
	thread = boost::thread(&logger_client::operator (), this);
}

logger_client::~logger_client()
{
//	cout<<"logger_client::~logger_client(): 1\n";
	terminate = true;
	{
		boost::mutex::scoped_lock lock(queue_mutex);
//		cout<<"logger_client::~logger_client(): 2\n";
		cond.notify_one();
//		cout<<"logger_client::~logger_client(): 3\n";
	}
//	cout<<"logger_client::~logger_client(): 4\n";
	thread.join();
//	cout<<"logger_client::~logger_client(): 5\n";
	disconnect();
//	cout<<"logger_client::~logger_client(): 6\n";
}

void logger_client::log(log_message& msg)
{
	msg.number = current_message_number;
	struct timespec tp;
	if (clock_gettime(CLOCK_REALTIME, &tp) != 0) {
		tp.tv_sec = tp.tv_nsec = 0;
	}
	msg.seconds = tp.tv_sec;
	msg.nanoseconds = tp.tv_nsec;

	boost::mutex::scoped_lock lock(queue_mutex);

	buffer.push_back(msg);

	current_message_number++;

	cond.notify_one();
}

void logger_client::operator()()
{
	try{
		queue_mutex.lock();
		boost::circular_buffer<log_message> temp_buffer(buffer.capacity());
		queue_mutex.unlock();

		connect();
		do {
			{
				boost::mutex::scoped_lock lock(queue_mutex);
				if(!buffer.empty() && !terminate){
					cond.wait(lock);
				}
				// move data from queue to temp buffer
				while(!buffer.empty()){
					temp_buffer.push_back(buffer.front());
					buffer.pop_front();
				}
			}

			// send data to the server
			while (!temp_buffer.empty()) {
				send_message(temp_buffer.front());
				temp_buffer.pop_front();
			}
		} while(!terminate);
		disconnect();
	}catch(std::exception& ex){
		cout<<"logger_client::operator()(): exception caught: "<< ex.what() <<"\n";
	}
	cout<<"logger_client::operator()(): exiting...\n";
}

void logger_client::connect()
{
	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
		throw runtime_error("socket(): " + string(strerror(errno)));
	}

	int flag = 1;
	if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) == -1) {

		throw runtime_error("setsockopt(): " + string(strerror(errno)));
	}

	hostent * server = gethostbyname(server_addr.c_str());
	if (server == NULL) {
		throw runtime_error("logger_client::connect(): gethostbyname(" + server_addr + "): " + string(hstrerror(h_errno)));
	}

	sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(server_port);

	if (::connect(fd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		throw runtime_error("logger_client::connect(): " + string(strerror(errno)));
	}
}

void logger_client::send_message(const log_message& msg)
{
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
		throw runtime_error("Socket::writev2() nwritten == -1");
	}
	if ((size_t) nwritten != iov[0].iov_len + iov[1].iov_len) {
		throw runtime_error("Socket::writev2() nwritten != buf1Size + buf2Size");
	}
}

void logger_client::disconnect()
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
}

} //namespace logger
