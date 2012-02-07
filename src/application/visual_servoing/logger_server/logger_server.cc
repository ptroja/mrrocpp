/*
 * logger_server.cc
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#include <iostream>

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <limits>
#include <cmath>
#include <cstring>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>

#include "logger_server.h"
#include "base/lib/logger_client/logger_client.h"

using namespace std;

namespace logger {

const int logger_server::default_port = 7000;

logger_server::logger_server(int port) :
		terminate_now(false), port(port), fd(-1), first_message_received(false)
{
	first_message_time.tv_sec = first_message_time.tv_nsec = 0;
}

logger_server::~logger_server()
{
	teardown_server();
}

void logger_server::setup_server()
{
	if ((fd = ::socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		throw runtime_error("socket() failed: " + string(strerror(errno)));
	}

	int on = 1;
	if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*) &on, sizeof(on)) < 0) {
		throw runtime_error("setsockopt() failed: " + string(strerror(errno)));
	}
	if (::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *) &on, sizeof(int))) {
		throw std::runtime_error("setsockopt(): " + std::string(strerror(errno)));
	}

	sockaddr_in m_addr;
	m_addr.sin_family = AF_INET;
	m_addr.sin_addr.s_addr = INADDR_ANY;
	m_addr.sin_port = htons(port);

	if (::bind(fd, (struct sockaddr*) &m_addr, sizeof(m_addr)) < 0) {
		throw runtime_error("bind() failed: " + string(strerror(errno)));
	}

	if (::listen(fd, 1) < 0) {
		throw runtime_error("listen() failed: " + string(strerror(errno)));
	}
}

void logger_server::teardown_server()
{
	if (fd >= 0) {
		::close(fd);
		fd = -1;
	}
}

void logger_server::accept_connection(){
	sockaddr_in m_addr;
	int addr_length = sizeof(m_addr);

	int acceptedFd = ::accept(fd, (sockaddr *) &m_addr, (socklen_t *) &addr_length);

	if (acceptedFd < 0) {
		throw runtime_error("accept() failed: " + string(strerror(errno)));
	}

	string client_address = inet_ntoa(m_addr.sin_addr);
	int client_port = ntohs(m_addr.sin_port);
	cout<<"client_address = "<<client_address<<":"<<client_port<<endl;

	stringstream ss;
	ss<<client_address<<"_"<<client_port;

	connections.push_back(boost::shared_ptr<client_connection>(new client_connection(this, acceptedFd, ss.str())));
}

void logger_server::main_loop()
{
	cout<<"logger_server::main_loop(): Starting...\n";
	fd_set rfds;
	struct timeval tv;

	while(!terminate_now){
		int maxFd = fd;
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		std::list<boost::shared_ptr<client_connection> >::iterator it;
		for(it = connections.begin(); it != connections.end(); ++it){
			FD_SET((*it)->connection_fd, &rfds);
			maxFd = max(maxFd, (*it)->connection_fd);
		}

		tv.tv_sec = 1;
		tv.tv_usec = 0;
		int ret = ::select(maxFd + 1, &rfds, NULL, NULL, &tv);
		if(ret < 0 && errno == EINTR){
			terminate_now = true;
		}else if(ret < 0){
			throw runtime_error("select() error: " + string(strerror(errno)));
		}else if(ret > 0){
			if(FD_ISSET(fd, &rfds)){ // accept new connection
				cout<<"New connection\n";
				accept_connection();
			}
			for(it = connections.begin(); it != connections.end();){
				try{
					if(FD_ISSET((*it)->connection_fd, &rfds)){
						(*it)->service();
					}
					++it;
				}catch(exception& ex){
					it = connections.erase(it);
				}
			}
			if(connections.size() == 0){
				first_message_received = false;
				cout<<"logger_server::main_loop(): Resetting t0...\n";
			}
		}
	}
	cout<<"logger_server::main_loop(): Terminating...\n";
}

void logger_server::run()
{
	setup_server();
	main_loop();
	teardown_server();
}

void logger_server::terminate()
{
	terminate_now = true;
}

double logger_server::calculate_message_time(const struct timespec &message_time)
{
	if(!first_message_received){
		first_message_time = message_time;
		first_message_received = true;
	}
	return time_diff(message_time, first_message_time);
}

} /* namespace logger */
