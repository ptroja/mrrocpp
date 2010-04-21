#include <iostream>
#include <exception>

#include <boost/foreach.hpp>
#include <boost/serialization/string.hpp>

#include "../messip/messip_dataport.h"
#include "../xdr_iarchive.hpp"

#include "DataBuffer.hh"

void Agent::registerBuffer(DataBufferBase & buf)
{
	std::pair<buffers_t::iterator, bool> result = buffers.insert(buffer_item_t(buf.getName(), &buf));

	if(!result.second) {
		std::cerr << "Duplicated name of the buffer ('" << buf.getName() << "')" << std::endl;
		throw;
	}
}

void Agent::listBuffers() const
{
	std::cout << "Buffer list of \"" << getName() << "\"[" << buffers.size() << "]:" << std::endl;
	BOOST_FOREACH(buffer_item_t item, buffers) {
		std::cout << "\t" << item.first << std::endl;
	}
}

Agent::Agent(const std::string & _name) : AgentBase(_name)
{
	channel = messip::port_create(getName());
}

Agent::~Agent()
{
	messip::port_delete(channel);
}

void Agent::operator ()()
{
	do {
	} while (step());
}

bool Agent::checkCondition(const OrBufferContainer &condition)
{
//	BOOST_FOREACH(const AndBufferContainer & andCondition, condition) {
//		std::cout << "(";
//		BOOST_FOREACH(const DataBufferBase * ptr, andCondition) {
//			std::cout << ptr->getName() << " & ";
//		}
//		std::cout << ") | ";
//	}

	BOOST_FOREACH(const AndBufferContainer & andCondition, condition) {
		if (andCondition.isFresh())
			return true;
	}
	return false;
}

bool Agent::ReceiveMessage(bool block)
{
	// loop to ignore protocol-level messages (i.e. 'connecting' message)
	while(true) {
		// buffer for the message to receive
		char msg[4096];

		int32_t type, subtype;
		int ret = messip_receive(channel, &type, &subtype, msg, sizeof(msg), (block) ? MESSIP_NOTIMEOUT : 0);

		if (!block & ret == MESSIP_MSG_TIMEOUT)
			return false;

		if (ret == -1) {
			std::cerr << "messip_receive() failed" << std::endl;
			throw;
		}
		else if (ret != MESSIP_MSG_NOREPLY) {
			std::cerr << "Unknown message received (" << ret << ")" << std::endl;
			continue;
		}

		xdr_iarchive<sizeof(msg)> ia(msg,channel->datalenr);

		// TODO: make this real-time safe
		std::string msg_buffer_name;

		ia >> msg_buffer_name;

		/*
		std::cout << "Message received for data buffer: "
			<< msg_buffer_name << ", size " << channel->datalenr << std::endl;
		*/

		buffers_t::iterator result = buffers.find(msg_buffer_name);
		if (result != buffers.end()) {
			result->second->Set(ia);
		} else {
			// TODO: exception
			throw;
		}

		return true;
	}
}

void Agent::Wait(OrBufferContainer & condition)
{
	while(true) {
		if(ReceiveMessage(false))
			continue;
		if(!checkCondition(condition))
			ReceiveMessage(true);
		else
			break;
	}
}

void Agent::Wait(AndBufferContainer & andCondition)
{
	OrBufferContainer orCondition(andCondition);
	Wait(orCondition);
}

void Agent::Wait(DataBufferBase & dataBufferCondition)
{
	OrBufferContainer orCondition(dataBufferCondition);
	Wait(orCondition);
}
