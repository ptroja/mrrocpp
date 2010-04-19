#ifndef _INT_WRITER_HH
#define _INT_WRITER_HH

#include <iostream>

#include <boost/thread/xtime.hpp>

#include "Agent.hh"
#include "RemoteAgent.hh"
#include "DataBuffer.hh"

class IntWriter : public Agent {
private:
	RemoteAgent reader;
	int cnt;
public:

	IntWriter(const std::string & name) :
		Agent(name), reader("Reader"), cnt(10)
	{
	}

	bool step() {
		std::cout << "Writer: " << cnt << std::endl;
		reader.Set("integer buffer", cnt++);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		return true;
	}
};

#endif /* _INT_WRITER_HH */
