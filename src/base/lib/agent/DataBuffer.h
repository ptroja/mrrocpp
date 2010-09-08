#ifndef _DATABUFFER_H
#define _DATABUFFER_H

#include <vector>
#include <ostream>

#include "base/lib/xdr/xdr_iarchive.hpp"

#include "Agent.h"

// forward declarations
class DataBufferBase;
class AndDataCondition;
class OrDataCondition;

template <class T>
class DataBuffer : public DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

private:
	//! current data
	T data;

	//! place for keeping new data after arrive
	T new_data;

	//! flag indicating that the new data has not been getted yet
	bool fresh;

	/**
	 * Store data in the buffer
	 * @param ia input archive
	 */
	void Store(xdr_iarchive<> & ia) {
 		ia >> new_data;
 		if (new_data_ready) {
 			std::cerr << "Warning: data overwrite at buffer '" << getName() << "'" << std::endl;
 		}
		new_data_ready = true;
	}

	/**
	 * Update data from the temporary to the current data buffer
	 */
	void Update(void) {
		if(new_data_ready) {
			data = new_data;
			new_data_ready = false;
			fresh = true;
		}
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
	T Get() {
		fresh = false;
		return data;
	}

	/**
	 * Get data from the buffer
	 * @param item where the data will be stored
	 * @return fresh flag indicatig if this data has been already "getted"
	 */
	bool Get(T & item) {
		bool fresh_flag = fresh;
		item = data;
		fresh = false;
		return fresh_flag;
	}

	/**
	 * Check if data has ben already "getted"
	 * @return fresh flag
	 */
	bool isFresh(void) const {
		return fresh;
	}
};

#endif /* _DATABUFFER_H */
