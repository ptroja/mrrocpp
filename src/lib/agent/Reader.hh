#include <iostream>

#include "Agent.hh"
#include "DataBuffer.hh"

class Reader : public Agent {
private:
	DataBuffer<int> IntBuffer;
	DataBuffer<double> DoubleBuffer;
public:

	Reader(const std::string & name) : Agent(name),
		IntBuffer(*this, "integer buffer"),
		DoubleBuffer(*this, "double buffer")
	{
		addBuffer(& IntBuffer);
		addBuffer(& DoubleBuffer);
		listBuffers();
	}

	bool step() {
		// Wait() << IntBuffer & DoubleBuffer;
		AndBufferContainer cont1 = (IntBuffer && DoubleBuffer);
		AndBufferContainer cont2 = cont1; //DoubleBuffer;
		AndBufferContainer cont3 = (cont1 && cont2);
//		OrBufferContainer or1 = IntBuffer || cont1;
//		cont3
		int i = IntBuffer.Get();
		double d = DoubleBuffer.Get();
		std::cout << "Reader: " << i << "," << d << std::endl;
		// Send() << IntBuffer & DoubleBuffer; //?
		return true;
	}
};
