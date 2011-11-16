#include <iostream>

#include "Agent.h"
#include "InputBuffer.h"

class Reader : public Agent {
public:
	InputBuffer<int> IntBuffer;
	InputBuffer<double> DoubleBuffer;

	Reader(const std::string & name) : Agent(name),
		IntBuffer("integer buffer"),
		DoubleBuffer("double buffer")
	{
		registerBuffer(IntBuffer);
		registerBuffer(DoubleBuffer);

		listBuffers();
	}
};
