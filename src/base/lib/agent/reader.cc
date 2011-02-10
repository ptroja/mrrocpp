#include <iostream>

#include "Reader.h"

int
main(int argc, char *argv[])
{
	Reader reader("Reader");

	for(;;) {
		reader.ReceiveSingleMessage(true);
		std::cout << "int: " << reader.IntBuffer.Get() << " double: " << reader.DoubleBuffer.Get() << std::endl;
	}

	return 0;
}
