#include <iostream>

#include "Agent.hh"
#include "DataBuffer.hh"

class Reader : public Agent {
private:
	DataBuffer<int> IntBuffer;
public:

	Reader(const std::string & name) : Agent(name),
		IntBuffer(*this, "integer buffer")
	{
		addBuffer(& IntBuffer);
		listBuffers();
	}

	bool step() {
		int i = IntBuffer.Get();
		std::cout << "Reader: " << i << std::endl;
		return true;
	}
};
