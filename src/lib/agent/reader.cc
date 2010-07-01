#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "Reader.h"

int
main(int argc, char *argv[])
{
	Reader reader("Reader");

	reader.Start();

	// ... the reader is executing

	reader.Join();

	return 0;
}
