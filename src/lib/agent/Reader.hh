#include <iostream>

#include <boost/array.hpp>

#include "Agent.hh"
#include "DataBuffer.hh"

class Reader : public Agent {
private:
	DataBuffer<int> IntBuffer;
	DataBuffer<double> DoubleBuffer;
	OrBufferContainer or1;
	int loops;
public:

	Reader(const std::string & name) : Agent(name),
		IntBuffer("integer buffer"),
		DoubleBuffer("double buffer")
	{
		registerBuffer(IntBuffer);
		registerBuffer(DoubleBuffer);
		or1 = DoubleBuffer | IntBuffer;
		listBuffers();
		loops = 0;
	}

	bool step() {
		Wait(or1);

		double d;
		bool double_fresh = DoubleBuffer.Get(d);
		std::cout << "Reader: " << IntBuffer.Get() << "," << d << std::endl;

		return true; //(loops++ < 10);
	}
};
