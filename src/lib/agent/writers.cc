#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "IntWriter.h"
#include "DoubleWriter.h"

int
main(int argc, char *argv[])
{
	IntWriter writer1("IntWriter");
	DoubleWriter writer2("DoubleWriter");

	writer1.Start();
	writer2.Start();

	// ... writers are executing

	writer1.Join();
	writer2.Join();

	return 0;
}
