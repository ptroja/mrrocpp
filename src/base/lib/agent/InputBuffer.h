#ifndef _INPUTBUFFER_H
#define _INPUTBUFFER_H

#include <boost/thread/thread_time.hpp>

#include "base/lib/xdr/xdr_iarchive.hpp"

#include "Agent.h"

/**
 * Input data buffer
 */
template <class T>
class InputBuffer : public InputBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

	//! Owner of the buffer
	Agent & owner;

private:
	//! current data
	T data;

	//! flag indicating that the new data has not been getted yet
	bool fresh;

	//! Timestamp of the last received message
	boost::system_time timestamp;

	/**
	 * Store data in the buffer
	 * @param ia input archive
	 */
	void Store(xdr_iarchive<> & ia)
	{
 		ia >> data;
 		if (fresh) {
 			std::cerr << "Warning: data overwrite at buffer '" << getName() << "'" << std::endl;
 		}
		fresh = true;

		// Record the timestamp
		timestamp = boost::get_system_time();
	}

public:
	//! Constructor
	InputBuffer(Agent & _owner, const std::string & _name, const T & _default_value = T())
		: InputBufferBase(_name), owner(_owner), data(_default_value),
		fresh(false), access(data)
	{
		owner.registerBuffer(*this);
	}

	//! Destructor
	~InputBuffer()
	{
		owner.unregisterBuffer(*this);
	}

	/**
	 * Get data from the buffer
	 * @return current data
	 */
	const T & Get() const
	{
		return data;
	}

	/**
	 * Set the data as used
	 */
	void markAsUsed()
	{
		fresh = false;
	}

	/**
	 * Check if data has been already "getted"
	 * @return fresh flag
	 */
	bool isFresh() const
	{
		return fresh;
	}

	/**
	 * Get the timestamp of the last message
	 */
	boost::system_time getTimestamp() const
	{
		return timestamp;
	}

	/**
	 * Read-only direct access to the data buffer
	 */
	const T & access;
};

#endif /* _INPUTBUFFER_H */
