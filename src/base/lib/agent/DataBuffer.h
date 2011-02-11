#ifndef _DATABUFFER_H
#define _DATABUFFER_H

#include <vector>
#include <ostream>

#include "base/lib/xdr/xdr_iarchive.hpp"

#include "Agent.h"

template <class T>
class DataBuffer : public DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

private:
	//! current data
	T data;

	//! flag indicating that the new data has not been getted yet
	bool fresh;

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
	}

public:
	//! Constructor
	DataBuffer(const std::string & _name, const T & _default_value = T())
		: DataBufferBase(_name), data(_default_value),
		fresh(false)
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
};

#endif /* _DATABUFFER_H */
