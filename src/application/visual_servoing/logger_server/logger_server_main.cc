#include <stdexcept>
#include <iostream>
#include <signal.h>
#include <unistd.h>

#include "logger_server.h"

using namespace std;

void handler(int signal);

static logger::logger_server *server;

int main(int argc, char *argv[])
{
	signal(SIGINT, handler);
	signal(SIGTERM, handler);

	try {
		server = new logger::logger_server;

		server->run();
	} catch (exception& ex) {
		cerr << "\n\nERROR: " << ex.what() << endl;
		return 1;
	}

	return 0;
}

void handler(int signal)
{
	psignal(signal, "Caught signal");
	if(server != NULL){
		server->terminate();
	}
}
