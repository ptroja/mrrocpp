#ifndef _INT_WRITER_HH
#define _INT_WRITER_HH

#include "Agent.hh"
#include "DataBuffer.hh"
#include <iostream>
#include <boost/thread/xtime.hpp>

class IntWriter : public Agent {
private:
	int cnt;
	Agent * reader;
public:

	IntWriter(const std::string & name) : Agent(name),	cnt(10)
	{
		reader = AgentFactory::getAgent("Reader");
	}

	bool step() {
		std::cout << "Writer: " << cnt << std::endl;
		reader->Set("integer buffer", cnt++);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		return true;
	}
};

#endif /* _INT_WRITER_HH */
