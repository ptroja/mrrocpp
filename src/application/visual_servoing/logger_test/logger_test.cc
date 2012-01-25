#include <iostream>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <unistd.h>

#include <boost/program_options.hpp>

#include "base/lib/logger_client/logger_client.h"


using namespace logger;
using namespace std;
namespace po = boost::program_options;

int main(int argc, char *argv[])
{
	int delay;
	int buffer_capacity;
	int messages_count;
	string server_name;
	string filename_prefix;
	int server_port;
	int number_of_reconnections;

	po::options_description desc("Allowed options");
	desc.add_options()
	    ("help", "produce help message")
	    ("delay", po::value<int>(&delay)->default_value(1e2), "delay between consecutive messages")
	    ("buffer_capacity", po::value<int>(&buffer_capacity)->default_value(30), "buffer_capacity")
	    ("messages_count", po::value<int>(&messages_count)->default_value(1000), "messages_count")
	    ("server_name", po::value<string>(&server_name)->default_value("localhost"), "server_name")
	    ("server_port", po::value<int>(&server_port)->default_value(7000), "server_port")
	    ("filename_prefix", po::value<string>(&filename_prefix)->default_value(""), "filename_prefix")
	    ("number_of_reconnections", po::value<int>(&number_of_reconnections)->default_value(3), "number_of_reconnections")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
	    cout << desc << "\n";
	    return 1;
	}

	cout<<"1\n";

	logger_client log(buffer_capacity, server_name.c_str(), server_port, "test;", filename_prefix);

	double min_time = INFINITY;
	double max_time = -INFINITY;
	double time_sum = 0.0;

	for(int j=0; j<number_of_reconnections; ++j){
		cout<<"Connecting... 1\n";
		log.set_connect();

		cout<<"Sending messages...\n";
		for(int i=0; i<messages_count; ++i){
			usleep(delay);
			log_message lm;
			stringstream ss;
			ss<<i;

			sprintf(lm.text, "Message %d", i);
			log.log(lm);
			struct timespec end_time;
			if (clock_gettime(CLOCK_REALTIME, &end_time) == 0) {
				struct timespec start_time;
				start_time.tv_sec = lm.seconds;
				start_time.tv_nsec = lm.nanoseconds;
				double t = time_diff(end_time, start_time);
				time_sum += t;
				min_time = min(min_time, t);
				max_time = max(max_time, t);
			} else {
				cerr<<"clock_gettime(CLOCK_REALTIME, &end_time) failed.\n";
			}
		}
		if(messages_count > 0){
			cout<<"avg_time = "<<(time_sum/messages_count)<<endl;
			cout<<"min_time = "<<min_time<<endl;
			cout<<"max_time = "<<max_time<<endl;
		}
		cout<<"All messages sent.\n";

		cout<<"Disconnecting...\n";
		log.set_disconnect();
		cout<<"Disconnected...\n";
	}


	return 0;
}

