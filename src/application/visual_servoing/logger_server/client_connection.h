/*
 * client_connection.h
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#ifndef CLIENT_CONNECTION_H_
#define CLIENT_CONNECTION_H_

#include <string>
#include <fstream>

#include "logger_server.h"

namespace logger {

class logger_server;

class client_connection
{
public:
	friend class logger_server;

	client_connection(int connection_fd, const std::string& remote_address);
	virtual ~client_connection();

	void service(logger_server* server);
private:
	client_connection(const client_connection&);
	int connection_fd;
	const std::string& remote_address;
	int header_size;

	int last_message_number;

	std::ofstream outFile;
};

} /* namespace logger */
#endif /* CLIENT_CONNECTION_H_ */
