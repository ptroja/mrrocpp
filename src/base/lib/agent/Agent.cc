#include <iostream>
#include <exception>

#include <boost/foreach.hpp>
#include <boost/serialization/string.hpp>

#include "base/lib/messip/messip.h"
#include "base/lib/xdr/xdr_iarchive.hpp"

#include "Agent.h"
#include "InputBuffer.h"

namespace mrrocpp {
namespace lib {
namespace agent {

Agent::Agent(const std::string & _name) :
	AgentBase(_name)
{
	channel = messip_channel_create(NULL, getName().c_str(), MESSIP_NOTIMEOUT, 0);

	if (channel == NULL) {
		throw std::runtime_error(getName() + "messip_channel_create failed()");
	}
}

Agent::~Agent()
{
	if (messip_channel_delete(channel, MESSIP_NOTIMEOUT) == -1)
	{
		std::cerr << "server channel delete for '" << getName() << "' failed" << std::endl;
	}
}

void Agent::registerBuffer(InputBufferBase & buf)
{
	std::pair <buffers_t::iterator, bool> result = buffers.insert(buffer_item_t(buf.getName(), &buf));

	if (!result.second) {
		throw std::logic_error(buf.getName() + ": error registering duplicate buffer");;
	}
}

void Agent::unregisterBuffer(InputBufferBase & buf)
{
	if (buffers.erase(buf.getName()) != 1) {
		throw std::logic_error(buf.getName() + ": error unregistering nonexistent buffer");
	}
}

void Agent::listBuffers() const
{
	std::cout << "Buffer list of \"" << getName() << "\"[" << buffers.size() << "]:" << std::endl;

	BOOST_FOREACH(const buffer_item_t item, buffers)
	{
		std::cout << "\t" << item.first << std::endl;
	}
}

int Agent::ReceiveMessage(void * msg, std::size_t msglen, bool block)
{
	// receive loop
	while (true) {
		// additional message type/subtype information
		int32_t type, subtype;

		int ret = messip_receive(channel, &type, &subtype, msg, msglen, block ? MESSIP_NOTIMEOUT : 0);

		if (ret == -1) {
			throw std::runtime_error("messip_receive failed()");
		} else if (!block && ret == MESSIP_MSG_TIMEOUT) {
			return -1;
		} else if (ret != MESSIP_MSG_NOREPLY) {
			//			std::cerr << "Unknown message received (" << ret << ")" << std::endl;
			continue;
		}

		return channel->datalenr;
	}
}

bool Agent::ReceiveSingleMessage(bool block)
{
	// create object for real-time usage in the loop
	// TODO: this should be private clas member to avoid memory allocation in real-time
	std::string msg_buffer_name;

	// allocate memory for the string object
	// TODO: this size should be #defined for the whole library
	msg_buffer_name.reserve(100);

	// buffer for the message to receive
	char msg[16384];

	int msglen = ReceiveMessage(msg, sizeof(msg), block);

	if(msglen >= 0) {
		// initialize the archive with the data received
		xdr_iarchive <sizeof(msg)> ia(msg, msglen);

		ia >> msg_buffer_name;

		Store(msg_buffer_name, ia);

		return true;
	}

	return false;
}

} // namespace agent
} // namespace lib
} // namespace mrrocpp
