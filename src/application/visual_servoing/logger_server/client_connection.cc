/*
 * client_connection.cpp
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#include "client_connection.h"

namespace logger {

client_connection::client_connection(int connection_fd, const std::string& remote_address):
		connection_fd(connection_fd), remote_address(remote_address)
{


}

client_connection::~client_connection()
{
	close(connection_fd);
}

void client_connection::service()
{

}

} /* namespace logger */
