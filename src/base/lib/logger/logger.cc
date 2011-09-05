	/**
 * \file logger.cc
 * \brief Logging utilities.
 * \bug Not multi-thread safe use of log_*_enabled flags
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#include <cstdio>
#include <cstdarg>
#include <time.h>

#include <sys/uio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "base/lib/mrmath/homog_matrix.h"

#include "logger.h"

using namespace std;

namespace logger {

bool log_enabled = true;
bool log_dbg_enabled = false;

void log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!log_enabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush( stdout);
	va_end(ap);
}

void log_dbg(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!log_dbg_enabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush( stdout);
	va_end(ap);
}

void print_hm(const mrrocpp::lib::Homog_matrix& hm)
{
	printf("[");
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
			printf("%f\t", hm(i, j));
		}
		if (i < 2) {
			printf("\t;\n");
		} else {
			printf("\t;\n");
		}
	}
	printf("]\n");
	fflush(stdout);
}

void log(const mrrocpp::lib::Homog_matrix& hm)
{
	if (log_enabled) {
		print_hm(hm);
	}
}

void log_dbg(const mrrocpp::lib::Homog_matrix& hm)
{
	if (log_dbg_enabled) {
		print_hm(hm);
	}
}

logger_client::logger_client(int max_queue_size, const char* server_addr, int server_port) :
		fd(-1), max_queue_size(max_queue_size), server_addr(server_addr), server_port(server_port), current_message_number(0), terminate(false)
{

	notify_mutex = boost::shared_ptr<boost::mutex> (new boost::mutex);
	queue_mutex = boost::shared_ptr<boost::mutex> (new boost::mutex);

	thread = boost::thread(&logger_client::operator (), this);
}

logger_client::~logger_client()
{
	cout<<"logger_client::~logger_client(): 1\n";
	terminate = true;
	cout<<"logger_client::~logger_client(): 2\n";
	notify_mutex->lock();
	notify_mutex->unlock();
	cout<<"logger_client::~logger_client(): 3\n";
	thread.join();
	cout<<"logger_client::~logger_client(): 4\n";
	disconnect();

	cout<<"logger_client::~logger_client(): 5\n";
	notify_mutex = boost::shared_ptr<boost::mutex> ((boost::mutex*)NULL);
	cout<<"logger_client::~logger_client(): 6\n";
	queue_mutex = boost::shared_ptr<boost::mutex> ((boost::mutex*)NULL);
	cout<<"logger_client::~logger_client(): 7\n";
}

void logger_client::log(const log_message& msg)
{
	queue_mutex->lock();
	if (queue.size() < max_queue_size) {
		log_message msg_to_queue = msg;
		msg_to_queue.number = current_message_number;
		struct timespec tp;
		if (clock_gettime(CLOCK_REALTIME, &tp) != 0) {
			tp.tv_sec = tp.tv_nsec = 0;
		}
		msg_to_queue.seconds = tp.tv_sec;
		msg_to_queue.nanoseconds = tp.tv_nsec;

		queue.push_back(msg_to_queue);
//		notify_mutex->unlock();
	}
	current_message_number++;
	queue_mutex->unlock();
}

void logger_client::operator()()
{
//	notify_mutex->lock();

	connect();
	while (!terminate) {
//		notify_mutex->lock();
		if (terminate) {
//			notify_mutex->unlock();
			break;
		}
		std::deque <log_message> temp;
		queue_mutex->lock();
		// move data from queue to temp buffer
		temp.swap(queue);
		queue.clear();
		queue_mutex->unlock();

		// send data to the server
		while (!temp.empty()) {
			send_message(temp.front());
			temp.pop_front();
		}
	}
	disconnect();
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

	hostent * server = gethostbyname(server_addr);
	if (server == NULL) {
		throw runtime_error(string("gethostbyname(") + server_addr + "): " + string(hstrerror(h_errno)));
	}

	sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(server_port);

	if (::connect(fd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		throw runtime_error("connect(): " + string(strerror(errno)));
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
