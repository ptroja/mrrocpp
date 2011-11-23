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

#include "base/lib/logger_client/log_message.h"

namespace logger {

class logger_server;

class client_connection
{
public:
	friend class logger_server;

	client_connection(logger_server* server, int connection_fd, const std::string& remote_address);
	virtual ~client_connection();

	void service();
private:
	client_connection(const client_connection&);

	log_message receive_message();
	void save_message(log_message& lm);

	logger_server* server;
	int connection_fd;
	const std::string& remote_address;

	int header_size;

	int last_message_number;

	std::ofstream outFile;
};

} /* namespace logger */
#endif /* CLIENT_CONNECTION_H_ */
