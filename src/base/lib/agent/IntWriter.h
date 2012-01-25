#ifndef _INT_WRITER_H
#define _INT_WRITER_H

#include <iostream>

#include <boost/thread/xtime.hpp>

#include "Agent.h"
#include "RemoteAgent.h"

#include "InputBuffer.h"
#include "OutputBuffer.h"

class IntWriter : public Agent {
private:
	RemoteAgent reader;
	OutputBuffer<int> IntBuf;
	int cnt;

public:

	IntWriter(const std::string & name) :
		Agent(name),
		reader("Reader"),
		IntBuf(reader, "integer buffer"),
		cnt(10)
	{
	}

	void operator()() {
		std::cout << "Writer: " << cnt << std::endl;
		//reader.Send("integer buffer", cnt++);
		IntBuf.Send(cnt++);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
};

#endif /* _INT_WRITER_H */
