#include "Agent.hh"
#include "DataBuffer.hh"
#include <iostream>
#include <boost/thread/xtime.hpp>

class Writer : public Agent {
private:
	int cnt;
	DataBuffer<int> *buf;
	Agent * reader;
public:

	Writer(const std::string & name) : Agent(name),	cnt(10)
	{
		reader = AgentFactory::getAgent("Reader");
		buf = reader->getBuffer<int>("integer buffer");
	}

	bool step() {
		std::cout << "Writer: " << cnt << std::endl;
//		boost::thread::sleep(1);
		buf->Set(cnt++);
		::sleep(1);
		return true;
	}
};
