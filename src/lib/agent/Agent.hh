#ifndef __AGENT_HH
#define __AGENT_HH

#include <map>
#include <string>

#include <boost/noncopyable.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

// forward declarations
class DataBufferBase;
template <class T> class DataBuffer;

#include "AgentFactory.hh"

/**
 * Agent base class
 */
class Agent : boost::noncopyable, boost::thread {
private:
	//! Agent's name
	const std::string name;

protected:
	//! Datatype of buffers container
	typedef std::map<const std::string, DataBufferBase * > buffers_t;

	//! Datatype of buffers container value
	typedef buffers_t::value_type buffer_item_t;

	//! Buffer container
	buffers_t buffers;

	//! Add a buffer to the agent
	void addBuffer(DataBufferBase * buf);

	//! List buffers of the agent
	void listBuffers() const;

	//! Get the pointer to buffer for write access
	template <class T>
	DataBuffer<T> * getBuffer(const std::string & name) {
		buffers_t::iterator result = buffers.find(name);
		if (result != buffers.end()) {
			return dynamic_cast<DataBuffer<T> *>(result->second);
		}
		// TODO: or throw an exception?
		return (DataBuffer<T> *) NULL;
	};

public:
	//! Condition variable for signalling new data in buffers
	boost::condition_variable data_condition_variable;

	//! Mutex for exclusive access to data buffers
	boost::mutex data_mutex;

public:
	//! Constructor
	Agent(const std::string & _name);

	//! Get name of the agent
	const std::string & getName() const;

	//! Single step of agent's transition function
	virtual bool step() = 0;

	//! Main loop of the agent
	void operator ()();
};

#endif /* __AGENT_HH */
