#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "AgentFactory.hh"
#include "Reader.hh"
#include "Writer.hh"

int
main(int argc, char *argv[])
{
	Reader reader("Reader");
	Writer writer("Writer");

	AgentFactory::listAgents();

	boost::thread t1(boost::bind(&Reader::operator(), &reader));
	boost::thread t2(boost::bind(&Writer::operator(), &writer));

	t1.join();
	t2.join();

	return 0;
}
