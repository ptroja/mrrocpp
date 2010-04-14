#ifndef _DOUBLE_WRITER_HH
#define _DOUBLE_WRITER_HH

#include "Agent.hh"
#include "DataBuffer.hh"
#include <iostream>
#include <boost/thread/xtime.hpp>

class DoubleWriter : public Agent {
private:
	double cnt;
	DataBuffer<double> *buf;
	Agent * reader;
public:

	DoubleWriter(const std::string & name) : Agent(name),	cnt(0.1)
	{
		reader = AgentFactory::getAgent("Reader");
		buf = reader->getBuffer<double>("double buffer");
	}

	bool step() {
		std::cout << "Writer: " << cnt << std::endl;
		buf->Set(cnt);
		cnt += 1.0;
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		return true;
	}
};

#endif /* _DOUBLE_WRITER_HH */
