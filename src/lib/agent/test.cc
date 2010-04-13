#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "AgentFactory.hh"
#include "Reader.hh"
#include "IntWriter.hh"
#include "DoubleWriter.hh"

int
main(int argc, char *argv[])
{
	Reader reader("Reader");
	IntWriter writer1("IntWriter");
	DoubleWriter writer2("DoubleWriter");

	AgentFactory::listAgents();

	boost::thread t1(boost::bind(&Reader::operator(), &reader));
	boost::thread t2(boost::bind(&IntWriter::operator(), &writer1));
	boost::thread t3(boost::bind(&DoubleWriter::operator(), &writer2));

	t1.join();
	t2.join();
	t3.join();

	return 0;
}
