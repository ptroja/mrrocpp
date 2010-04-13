#ifndef _INT_WRITER_HH
#define _INT_WRITER_HH

#include "Agent.hh"
#include "DataBuffer.hh"
#include <iostream>
#include <boost/thread/xtime.hpp>

class IntWriter : public Agent {
private:
	int cnt;
	DataBuffer<int> *buf;
	Agent * reader;
public:

	IntWriter(const std::string & name) : Agent(name),	cnt(10)
	{
		reader = AgentFactory::getAgent("Reader");
		buf = reader->getBuffer<int>("integer buffer");
	}

	bool step() {
		std::cout << "Writer: " << cnt << std::endl;
		buf->Set(cnt++);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		return true;
	}
};

#endif /* _INT_WRITER_HH */
