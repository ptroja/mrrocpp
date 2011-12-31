#ifndef __AGENT_H
#define __AGENT_H

#include <string>

#include <boost/unordered_map.hpp>

#include "base/lib/messip/messip.h"

#include "base/lib/xdr/xdr_iarchive.hpp"
#include "AgentBase.h"

#include "InputBufferBase.h"

namespace mrrocpp {
namespace lib {
namespace agent {

/**
 * Base class for every agent
 */
class Agent : public AgentBase
{
public:
	//! Constructor
	Agent(const std::string & _name);

	//! Destructor
	virtual ~Agent();

	//! Receive single message
	//! @param block true for blocking mode
	//! @return true if a message has been received
	bool ReceiveSingleMessage(bool block);

protected:
	//! Datatype of buffers container
	typedef boost::unordered_map <std::string, InputBufferBase *> buffers_t;

	//! Datatype of buffers container value
	typedef buffers_t::value_type buffer_item_t;

	//! Buffer container
	buffers_t buffers;

	//! List buffers of the agent
	void listBuffers() const;

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

	//! Give access to buffer registration
	friend class InputBufferBase;

	//! Add a buffer to the agent
	void registerBuffer(InputBufferBase & buf);

	//! Remove buffer
	void unregisterBuffer(InputBufferBase & buf);
};

} // namespace agent
} // namespace lib
} // namespace mrrocpp

#endif /* __AGENT_H */
