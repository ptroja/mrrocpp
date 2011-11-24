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

#include "base/lib/logger_client/log_message.h"

using namespace std;

namespace logger {

client_connection::client_connection(int connection_fd, const std::string& remote_address) :
	connection_fd(connection_fd), remote_address(remote_address), last_message_number(-1)
{
	xdr_oarchive <> oa;
	oa << log_message_header();
	header_size = oa.getArchiveSize();

	time_t timep = time(NULL);
	struct tm* time_split = localtime(&timep);
	char time_log_filename[128];
	sprintf(time_log_filename, "../../msr/%04d-%02d-%02d_%02d-%02d-%02d_%s.csv", time_split->tm_year + 1900, time_split->tm_mon
			+ 1, time_split->tm_mday, time_split->tm_hour, time_split->tm_min, time_split->tm_sec, remote_address.c_str());

	outFile.open(time_log_filename, ofstream::out | ofstream::trunc);
}

client_connection::~client_connection()
{
	cout << "client_connection::~client_connection() (" << connection_fd << "): disconnected\n";
	outFile.close();
	close(connection_fd);
}

void client_connection::service(logger_server* server)
{
	//	cout << "client_connection::service(" << connection_fd << "):\n";
	xdr_iarchive <> ia;
	if (read(connection_fd, ia.get_buffer(), header_size) != header_size) {
		throw runtime_error("read() != header_size");
	}
	log_message_header lmh;
	ia >> lmh;

	//	cout << "    lmh.message_size = " << lmh.message_size << endl;

	ia.clear_buffer();
	if (read(connection_fd, ia.get_buffer(), lmh.message_size) != lmh.message_size) {
		throw runtime_error("read() != lmh.message_size");
	}

	log_message lm;
	ia >> lm;

	struct timespec message_time;

	message_time.tv_nsec = lm.nanoseconds;
	message_time.tv_sec = lm.seconds;
	double message_time_s = server->calculate_message_time(message_time);

	if (last_message_number + 1 != lm.number) {
		cerr << "!!!!!!!!!!!!!!!!!!!Buffer overflow detected!!!!!!!!!!!!!!!!!!!!!!\n";
	}

	//	cout << "    " << lm.number << ";" << lm.seconds << ";" << lm.nanoseconds << ";" << message_time_s << "\n    "
	//			<< lm.text << endl;

	outFile << lm.number << ";" <<message_time_s <<";";
	for(int i=0; i<lm.time_elems; ++i){
		struct timespec ts = lm.time_buf[i];
		if(ts.tv_sec == 0){
			outFile << ";";
		}else{
			double t = server->calculate_message_time(ts);
			outFile.precision(9);
			outFile << t << ";";
		}
	}
	outFile << lm.text << endl;

	last_message_number = lm.number;
}

} /* namespace logger */
