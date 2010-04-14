#include <iostream>

#include <boost/foreach.hpp>

#include "DataBuffer.hh"

void Agent::addBuffer(DataBufferBase * buf)
{
	// TODO: check for result
	buffers.insert(buffer_item_t(buf->getName(), buf));
}

void Agent::listBuffers() const
{
	std::cout << "Buffer list of \"" << getName() << "\"[" << buffers.size() << "]:" << std::endl;
	BOOST_FOREACH(buffer_item_t item, buffers) {
		std::cout << "\t" << item.first << std::endl;
	}
}

Agent::Agent(const std::string & _name) : name(_name)
{
	AgentFactory::addAgent(this);
}

const std::string & Agent::getName() const
{
	return name;
}

void Agent::operator ()()
{
	do {
	} while (step());
}

bool Agent::checkCondition(const OrBufferContainer &condition)
{
	BOOST_FOREACH(const AndBufferContainer & andCondition, condition) {
		if (andCondition.isFresh())
			return true;
	}
	return false;
}

void Agent::Wait(OrBufferContainer & condition)
{
	boost::unique_lock <boost::mutex> lock(buffers_mutex);
	while (!checkCondition(condition)) {
		data_condition_variable.wait(lock);
	}
}
