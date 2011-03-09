/*
 * DataBufferBase.h
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#ifndef DATABUFFERBASE_H_
#define DATABUFFERBASE_H_

#include <string>

#include "base/lib/xdr/xdr_iarchive.hpp"

class DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

protected:
	//! name of the data buffer
	const std::string name;

	//! store new data
	virtual void Store(xdr_iarchive<> & ia) = 0;

public:
	//! Constructor
	DataBufferBase(const std::string & _name);

	//! get name of the buffer
	const std::string & getName() const;

	//! This is required to make a class polimorphic
	virtual ~DataBufferBase();
};

#endif /* DATABUFFERBASE_H_ */
