#ifndef _DATABUFFER_H
#define _DATABUFFER_H

#include <vector>
#include <ostream>

#include <boost/thread/thread_time.hpp>

#include "base/lib/xdr/xdr_iarchive.hpp"

#include "Agent.h"

template <class T>
class InputBuffer : public DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

private:
	//! current data
	T data;

	//! flag indicating that the new data has not been getted yet
	bool fresh;

	//! Timestamp of the last message received
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
	InputBuffer(const std::string & _name, const T & _default_value = T())
		: DataBufferBase(_name), data(_default_value),
		fresh(false), access(data)
	{
	}

	/**
	 * Get data from the buffer
	 * @return current data
	 */
	T Get() const
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
	boost::system_time getTimestamp()
	{
		return timestamp;
	}

	/**
	 * Read-only direct access to the data buffer
	 */
	const T & access;
};

#endif /* _DATABUFFER_H */
