#include <iostream>

#include "Agent.h"
#include "DataBuffer.h"

class Reader : public Agent {
public:
	DataBuffer<int> IntBuffer;
	DataBuffer<double> DoubleBuffer;

	Reader(const std::string & name) : Agent(name),
		IntBuffer("integer buffer"),
		DoubleBuffer("double buffer")
	{
		registerBuffer(IntBuffer);
		registerBuffer(DoubleBuffer);

		listBuffers();
	}
};
