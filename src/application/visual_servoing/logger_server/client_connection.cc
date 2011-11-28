/*
 * client_connection.cpp
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#include <stdexcept>
#include <iostream>

#include "client_connection.h"

using namespace std;

namespace logger {

client_connection::client_connection(logger_server* server, int connection_fd, const std::string& remote_address) :
	server(server), connection_fd(connection_fd), remote_address(remote_address), last_message_number(-1)
{
	xdr_oarchive <> oa;
	oa << log_message_header();
	header_size = oa.getArchiveSize();

	config_message cm = receive_message<config_message>();

	time_t timep = time(NULL);
	struct tm* time_split = localtime(&timep);
	sprintf(time_log_filename, "../../msr/%s_%04d-%02d-%02d_%02d-%02d-%02d_%s.csv", cm.filename_prefix, time_split->tm_year + 1900, time_split->tm_mon
			+ 1, time_split->tm_mday, time_split->tm_hour, time_split->tm_min, time_split->tm_sec, remote_address.c_str());

	outFile.open(time_log_filename, ofstream::out | ofstream::trunc);
	outFile << "message_number;message_time_s;"<<cm.header<<"\n";
}

client_connection::~client_connection()
{
	cout << "client_connection::~client_connection() (" << remote_address << ", " <<  time_log_filename <<"): disconnected\n";
	outFile.close();
	close(connection_fd);
}

void client_connection::service()
{
	//	cout << "client_connection::service(" << connection_fd << "):\n";
	log_message lm = receive_message<log_message>();

	if (last_message_number + 1 != lm.number) {
		cerr << "!!!!!!!!!!!!!!!!!!!logger_client buffer overflow detected!!!!!!!!!!!!!!!!!!!!!!\n";
	}

	save_message(lm);

	last_message_number = lm.number;
}



void client_connection::save_message(log_message& lm)
{
	struct timespec message_time;

	message_time.tv_nsec = lm.nanoseconds;
	message_time.tv_sec = lm.seconds;
	double message_time_s = server->calculate_message_time(message_time);

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
}

} /* namespace logger */
