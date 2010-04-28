#ifndef _INT_WRITER_H
#define _INT_WRITER_H

#include <iostream>

#include <boost/thread/xtime.hpp>

#include "Agent.h"
#include "RemoteAgent.h"
#include "DataBuffer.h"

class IntWriter : public Agent {
private:
	RemoteAgent reader;
	RemoteBuffer<int> IntBuf;
	int cnt;
public:

	IntWriter(const std::string & name) :
		Agent(name),
		reader("Reader"),
		IntBuf(reader, "integer buffer"),
		cnt(10)
	{
	}

	bool step() {
		std::cout << "Writer: " << cnt << std::endl;
		//reader.Set("integer buffer", cnt++);
		IntBuf.Set(cnt++);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		return true;
	}
};

#endif /* _INT_WRITER_H */
