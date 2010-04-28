#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "IntWriter.hh"
#include "DoubleWriter.hh"

int
main(int argc, char *argv[])
{
	IntWriter writer1("IntWriter");
	DoubleWriter writer2("DoubleWriter");

	boost::thread t2(boost::bind(&IntWriter::operator(), &writer1));
	boost::thread t3(boost::bind(&DoubleWriter::operator(), &writer2));

	t2.join();
	t3.join();

	return 0;
}
