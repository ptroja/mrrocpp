#ifndef __AGENT_H
#define __AGENT_H

#include <string>

#include <boost/unordered_map.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "base/lib/messip/messip.h"

#include "base/lib/xdr/xdr_iarchive.hpp"
#include "AgentBase.h"

#include "DataBufferBase.h"
#include "DataBuffer.h"

/**
 * Agent base class
 */
class Agent : public AgentBase
{
private:
	//! server channel id
	messip_channel_t * channel;

	//! Store data from archive into a buffer
	template <std::size_t size>
	void Store(const std::string & buffer_name, xdr_iarchive <size> & ia)
	{
		//	std::cout << "Message received for data buffer: "
		//		<< msg_buffer_name << ", size "
		//#if (USE_MESSIP_SRR)
		//		<< channel->datalenr << std::endl;
		//#else
		//		<< info.msglen << std::endl;
		//#endif

		buffers_t::iterator result = buffers.find(buffer_name);
		if (result != buffers.end()) {
			result->second->Store(ia);
		} else {
			// TODO: exception?
			std::cerr << "Message received for unknown buffer '" << buffer_name << "'" << std::endl;
		}
	}

	//! Receive single message
	int ReceiveMessage(void * msg, std::size_t msglen, bool block);

protected:
	//! Datatype of buffers container
	typedef boost::unordered_map <std::string, DataBufferBase *> buffers_t;

	//! Datatype of buffers container value
	typedef buffers_t::value_type buffer_item_t;

	//! Buffer container
	buffers_t buffers;

	//! Add a buffer to the agent
	void registerBuffer(DataBufferBase & buf);

	//! List buffers of the agent
	void listBuffers() const;

public:
	//! Constructor
	Agent(const std::string & _name);

	//! Receive single message
	//! @param block true for blocking mode
	//! @return true if a message has been received
	bool ReceiveSingleMessage(bool block);

	//! Destructor
	virtual ~Agent();
};

#endif /* __AGENT_H */
