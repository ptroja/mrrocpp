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

#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"


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

	template<typename T>
	T receive_message();
	void save_message(log_message& lm);

	logger_server* server;
	int connection_fd;
	const std::string remote_address;

	int header_size;

	int last_message_number;

	std::ofstream outFile;

	char time_log_filename[2048];
};

template<typename T>
T client_connection::receive_message()
{
	xdr_iarchive <> ia;
	if (read(connection_fd, ia.get_buffer(), header_size) != header_size) {
		throw std::runtime_error("read() != header_size");
	}
	log_message_header lmh;
	ia >> lmh;

	//	cout << "    lmh.message_size = " << lmh.message_size << endl;

	ia.clear_buffer();
	if (read(connection_fd, ia.get_buffer(), lmh.message_size) != lmh.message_size) {
		throw std::runtime_error("read() != lmh.message_size");
	}

	T lm;
	ia >> lm;
	return lm;
}

} /* namespace logger */
#endif /* CLIENT_CONNECTION_H_ */
