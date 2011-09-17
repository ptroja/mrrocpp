/*
 * client_connection.cpp
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#include <stdexcept>
#include <iostream>

#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"

#include "client_connection.h"

#include "base/lib/logger/log_message.h"

using namespace std;

namespace logger {

client_connection::client_connection(int connection_fd, const std::string& remote_address):
		connection_fd(connection_fd), remote_address(remote_address), last_message_number(-1)
{
	xdr_oarchive<> oa;
	oa<<log_message_header();
	header_size = oa.getArchiveSize();
}

client_connection::~client_connection()
{
	close(connection_fd);
}

void client_connection::service()
{
	cout<<"client_connection::service("<<connection_fd<<"):\n";
	xdr_iarchive<> ia;
	if(read(connection_fd, ia.get_buffer(), header_size) != header_size){
		throw runtime_error("read() != header_size");
	}
	log_message_header lmh;
	ia >> lmh;

	cout<<"    lmh.message_size = "<<lmh.message_size<<endl;

	ia.clear_buffer();
	if(read(connection_fd, ia.get_buffer(), lmh.message_size) != lmh.message_size){
		throw runtime_error("read() != lmh.message_size");
	}

	log_message lm;
	ia >> lm;

	if(last_message_number + 1 != lm.number){
		cerr<<"!!!!!!!!!!!!!!!!!!!Buffer overflow detected!!!!!!!!!!!!!!!!!!!!!!\n";
	}

	cout<<"    "<<lm.number<<";"<<lm.seconds<<";"<<lm.nanoseconds<<"\n    "<<lm.text<<endl;
	last_message_number = lm.number;
}

} /* namespace logger */
