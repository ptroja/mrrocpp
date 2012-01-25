#ifndef _INPUTBUFFER_H
#define _INPUTBUFFER_H

#include <boost/thread/thread_time.hpp>

#include "InputBufferBase.h"
#include "base/lib/xdr/xdr_iarchive.hpp"

namespace mrrocpp {
namespace lib {
namespace agent {

/**
 * Input data buffer
 */
template <class T>
class InputBuffer : public InputBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

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
		: InputBufferBase(_owner, _name), data(_default_value),
		fresh(false), access(data)
	{
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

} // namespace agent
} // namespace lib
} // namespace mrrocpp

#endif /* _INPUTBUFFER_H */
