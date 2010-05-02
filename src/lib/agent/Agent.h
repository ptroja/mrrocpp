#ifndef __AGENT_H
#define __AGENT_H

#include <string>

#include <boost/unordered_map.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip.h"
#else /* USE_MESSIP_SRR */
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif /* USE_MESSIP_SRR */

#include "lib/xdr_iarchive.hpp"
#include "AgentBase.h"

#include "DataBufferBase.h"
#include "DataBuffer.h"
#include "OrDataCondition.h"
#include "AndDataCondition.h"

/**
 * Agent base class
 */
class Agent : public AgentBase {
private:
	//! check if given data availability condition is satisfied
	bool checkCondition(const OrDataCondition &condition);

#if defined(USE_MESSIP_SRR)
	//! server channel id
	messip_channel_t * channel;
#else
	//! server channel id
	name_attach_t * channel;
#endif

	//! thread id of the of the non-blocking receive implementation
	boost::thread * tid;

	//! condition variable for synchronization wake-up after receiving data
	boost::condition_variable cond;

	//! mutex for protection data between receiver and readers
	mutable boost::mutex mtx;

	//! Data receiver thread loop
	void ReceiveDataLoop(void);

	//! Store data from archive into a buffer
	void Store(const std::string & buffer_name, xdr_iarchive<> & ia);

	//! Receive single message
	int ReceiveMessage(void * msg, std::size_t msglen, bool block);

	//! thread id of the of the non-blocking receive implementation
	boost::thread loop_tid;

	//! Main loop of the agent
	void operator ()();

protected:
	//! Datatype of buffers container
	typedef boost::unordered_map<std::string, DataBufferBase * > buffers_t;

	//! Datatype of buffers container value
	typedef buffers_t::value_type buffer_item_t;

	//! Buffer container
	buffers_t buffers;

	//! Add a buffer to the agent
	void registerBuffer(DataBufferBase & buf);

	//! List buffers of the agent
	void listBuffers() const;

protected:
	/**
	 * Wait for given data availability condition to be satisfied
	 * @param condition condition to wait for
	 */
	void Wait(DataCondition & condition);

	/**
	 * Wait for null data availability condition to be satisfied
	 */
	void Wait(void);

public:
	//! Constructor
	Agent(const std::string & _name);

	//! Destructor
	virtual ~Agent();

	//! Single step of agent's transition function
	virtual bool step() = 0;

	//! Start the agent
	void Start(void);

	//! Join the agent
	void Join(void);
};

#endif /* __AGENT_H */
