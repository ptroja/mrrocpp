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

logger_client::logger_client(int max_queue_size, const char* server_addr, const char* server_port) :
		max_queue_size(max_queue_size), server_addr(server_addr), server_port(server_port), current_message_number(0), terminate(false)
{
	notify_mutex.lock();

	thread = boost::thread(&logger_client::operator (), this);
}

logger_client::~logger_client()
{
	terminate = true;
	notify_mutex.unlock();
	thread.join();
}

void logger_client::log(const log_message* msg)
{
	queue_mutex.lock();
	if (queue.size() < max_queue_size) {
		log_message* msg_to_queue = msg->clone();
		msg_to_queue->number = current_message_number;
		struct timespec tp;
		if(clock_gettime(CLOCK_REALTIME, &tp) != 0){
			tp.tv_sec = tp.tv_nsec = 0;
		}
		msg_to_queue->seconds = tp.tv_sec;
		msg_to_queue->nanoseconds = tp.tv_nsec;


		queue.push_back(msg_to_queue);
		notify_mutex.unlock();
	}
	current_message_number++;
	queue_mutex.unlock();
}

void logger_client::operator()()
{
	connect();
	while (!terminate) {
		notify_mutex.lock();
		if (terminate) {
			break;
		}
		std::deque <const log_message*> temp;
		queue_mutex.lock();
		// move data from queue to temp buffer
		temp.assign(queue.begin(), queue.end());
		queue.clear();
		queue_mutex.unlock();

		// send data to the server
	}
	disconnect();
}

void logger_client::connect()
{

}

void logger_client::send_message(const log_message* msg)
{

}

void logger_client::disconnect()
{

}

} //namespace logger
