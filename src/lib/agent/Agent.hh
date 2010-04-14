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
class OrBufferContainer;

#include "AgentFactory.hh"

/**
 * Agent base class
 */
class Agent : boost::noncopyable, boost::thread {
private:
	//! Agent's name
	const std::string name;

	//! check if given data availability condition is satisfied
	bool checkCondition(const OrBufferContainer &condition);

	//! Condition variable for signalling new data in buffers
	boost::condition_variable data_condition_variable;

	//! Mutex for exclusive access to data buffers
	boost::mutex buffers_mutex;

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

	/**
	 * Get the data from given buffer
	 * @param name buffer name
	 * @return the data
	 */
	template <class T>
	T Get(const std::string & name) {
		boost::mutex::scoped_lock lock(buffers_mutex);
		buffers_t::iterator result = buffers.find(name);
		if (result != buffers.end()) {
			DataBuffer<T> * buffer = dynamic_cast<DataBuffer<T> *>(result->second);
			return buffer->Get();
		}
		// TODO: exception
		throw;
	};

	/**
	 * Set the data of given buffer
	 * @param name buffer name
	 * @param the data
	 */
	template <class T>
	void Set(const std::string & name, const T & item) {
		boost::mutex::scoped_lock lock(buffers_mutex);
		buffers_t::iterator result = buffers.find(name);
		if (result != buffers.end()) {
			DataBuffer<T> * buffer = dynamic_cast<DataBuffer<T> *>(result->second);
			data_condition_variable.notify_one();
			return buffer->Set(item);
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
