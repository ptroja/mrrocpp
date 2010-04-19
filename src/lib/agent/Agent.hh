#ifndef __AGENT_HH
#define __AGENT_HH

#include <map>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

// forward declarations
class DataBufferBase;
template <class T> class DataBuffer;
class OrBufferContainer;
class AndBufferContainer;

#include "../messip/messip_dataport.h"

#include "AgentBase.hh"

/**
 * Agent base class
 */
class Agent : public AgentBase {
private:
	//! check if given data availability condition is satisfied
	bool checkCondition(const OrBufferContainer &condition);

	/**
	 * Receive data buffer message
	 * @param block until a message arrive
	 * @return true if the a message has arrived
	 */
	bool ReceiveMessage(bool block);

	//! messip channel object
	messip_channel_t * channel;

protected:
	//! Datatype of buffers container
	typedef std::map<const std::string, DataBufferBase * > buffers_t;

	//! Datatype of buffers container value
	typedef buffers_t::value_type buffer_item_t;

	//! Buffer container
	buffers_t buffers;

	//! Add a buffer to the agent
	void registerBuffer(DataBufferBase & buf);

	//! List buffers of the agent
	void listBuffers() const;

	/**
	 * Get the data from given buffer
	 * @param name buffer name
	 * @return the data
	 */
	template <class T>
	T Get(const std::string & name) {
		buffers_t::iterator result = buffers.find(name);
		if (result != buffers.end()) {
			DataBuffer<T> * buffer = dynamic_cast<DataBuffer<T> *>(result->second);
			return buffer->Get();
		}
		// TODO: exception
		throw;
	};

protected:
	/**
	 * Wait for given data availability condition to be satisfied
	 * @param orCondition condition to wait for
	 */
	void Wait(OrBufferContainer & orCondition);

	void Wait(AndBufferContainer & orCondition);

	void Wait(DataBufferBase & dataBufferCondition);

public:
	//! Constructor
	Agent(const std::string & _name);

	//! Destructor
	virtual ~Agent();

	//! Single step of agent's transition function
	virtual bool step() = 0;

	//! Main loop of the agent
	void operator ()();
};

#endif /* __AGENT_HH */
