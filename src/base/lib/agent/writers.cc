#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "IntWriter.h"
#include "DoubleWriter.h"

int
main(int argc, char *argv[])
{
	IntWriter writer1("IntWriter");
	DoubleWriter writer2("DoubleWriter");

	while(true) {
		writer1();
		writer2();
	}

	return 0;
}
