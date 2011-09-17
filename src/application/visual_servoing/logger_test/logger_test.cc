#include <iostream>
#include <sstream>
#include <unistd.h>

#include <boost/program_options.hpp>

#include "base/lib/logger/logger.h"


using namespace logger;
using namespace std;
namespace po = boost::program_options;

int main(int argc, char *argv[])
{
	int delay;
	int buffer_capacity;
	int messages_count;
	string server_name;
	int server_port;

	po::options_description desc("Allowed options");
	desc.add_options()
	    ("help", "produce help message")
	    ("delay", po::value<int>(&delay)->default_value(1e2), "delay between consecutive messages")
	    ("buffer_capacity", po::value<int>(&buffer_capacity)->default_value(3), "buffer_capacity")
	    ("messages_count", po::value<int>(&messages_count)->default_value(1000), "messages_count")
	    ("server_name", po::value<string>(&server_name)->default_value("localhost"), "server_name")
	    ("server_port", po::value<int>(&server_port)->default_value(7000), "server_port")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	cout<<"1\n";

	logger_client log(buffer_capacity, server_name.c_str(), server_port);

	for(int i=0; i<messages_count; ++i){
		usleep(delay);
		log_message lm;
		stringstream ss;
		ss<<i;

		sprintf(lm.text, "Message %d", i);
		log.log(lm);
	}
	cout<<"2\n";

	return 0;
}

