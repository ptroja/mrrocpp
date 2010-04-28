#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "Reader.hh"

int
main(int argc, char *argv[])
{
	Reader reader("Reader");

	boost::thread t1(boost::bind(&Reader::operator(), &reader));
	t1.join();

	return 0;
}
