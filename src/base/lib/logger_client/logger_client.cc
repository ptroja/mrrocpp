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

logger_client::logger_client(int buffer_size, const std::string& server_addr, int server_port, const std::string& header_text, const std::string& filename_prefix) :
		fd(-1), buffer(buffer_size), server_addr(server_addr), server_port(server_port), current_message_number(0), terminate(false), connect_now(false), disconnect_now(false), connected(false), header_text(header_text), filename_prefix(filename_prefix)
{
	thread = boost::thread(&logger_client::operator (), this);
}

logger_client::~logger_client()
{
	terminate = true;
	{
		boost::mutex::scoped_lock lock(queue_mutex);
		cond.notify_one();
	}
	thread.join();
	disconnect();
}

void logger_client::log(log_message& msg)
{
	if(!connected){
		return;
	}
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

		do {
			{
				boost::mutex::scoped_lock lock(queue_mutex);
				if(!buffer.empty() && !terminate && !connect_now && !disconnect_now){
					cond.wait(lock);
				}
				if(connect_now){
					try{
						buffer.clear();
						connect();
					}catch(exception& ex){
						// do nothing
					}
					connect_now = false;
					connect_cond.notify_one();
				} else if(disconnect_now){
					//printf("logger_client::operator()(): disconnect_now == true 1");
					buffer.clear();
					disconnect();
					//printf("logger_client::operator()(): disconnect_now == true 2");
					disconnect_now = false;
					disconnect_cond.notify_one();
					//printf("logger_client::operator()(): disconnect_now == true 3");
				} else{
					// move data from queue to temp buffer
					while(!buffer.empty()){
						temp_buffer.push_back(buffer.front());
						buffer.pop_front();
					}
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
	current_message_number = 0;

	config_message cm;
	strcpy(cm.header, header_text.c_str());
	strcpy(cm.filename_prefix, filename_prefix.c_str());
	send_message(cm);
	connected = true;
}

void logger_client::disconnect()
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
	connected = false;
}

void logger_client::set_filename_prefix(const std::string& filename_prefix)
{
	this->filename_prefix = filename_prefix;
}

void logger_client::set_connect()
{
	//printf("logger_client::set_connect() 1\n");
	boost::mutex::scoped_lock lock(queue_mutex);
	//printf("logger_client::set_connect() 2\n");
	connect_now = true;
	cond.notify_one();
	//printf("logger_client::set_connect() 3\n");
	//while(!connected){
		//printf("logger_client::set_connect() 5\n");
		connect_cond.wait(lock);
	//}
	//printf("logger_client::set_connect() 6\n");
}
void logger_client::set_disconnect()
{
	//printf("logger_client::set_disconnect() 1\n");
	boost::mutex::scoped_lock lock(queue_mutex);
	//printf("logger_client::set_disconnect() 2\n");
	disconnect_now = true;
	cond.notify_one();
	//printf("logger_client::set_disconnect() 3\n");
	while(connected){
		//printf("logger_client::set_disconnect() 5\n");
		disconnect_cond.wait(lock);
	}
	//printf("logger_client::set_disconnect() 6\n");
}

} //namespace logger
