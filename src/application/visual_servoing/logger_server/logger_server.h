/*
 * logger_server.h
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#ifndef LOGGER_SERVER_H_
#define LOGGER_SERVER_H_

#include <list>
#include "client_connection.h"

namespace logger {

class logger_server
{
public:
	logger_server(int port = default_port);
	virtual ~logger_server();

	void run();

	void terminate();

	static const int default_port;
protected:

private:
	void setup_server();
	void teardown_server();
	void accept_connection();
	void main_loop();

	bool terminate_now;
	const int port;
	int fd;
	std::list<client_connection> connections;
};

} /* namespace logger */
#endif /* LOGGER_SERVER_H_ */
