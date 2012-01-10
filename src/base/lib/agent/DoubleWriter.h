#ifndef _DOUBLE_WRITER_H
#define _DOUBLE_WRITER_H

#include <iostream>

#include <boost/thread/xtime.hpp>

#include "RemoteAgent.h"
#include "InputBuffer.h"
#include "OutputBuffer.h"

class DoubleWriter : public Agent
{
private:
	RemoteAgent reader;
	OutputBuffer<double> DoubleBuf;
	double cnt;
public:

	DoubleWriter(const std::string & name) :
		Agent(name),
		reader("Reader"),
		DoubleBuf(reader, "double buffer"),
		cnt(0.1)
	{
	}

	void operator()()
	{
		std::cout << "Writer: " << cnt << std::endl;
		DoubleBuf.Send(cnt);
		cnt += 1.0;
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
};

#endif /* _DOUBLE_WRITER_H */
