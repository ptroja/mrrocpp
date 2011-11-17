/*
 * InputBufferBase.h
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#ifndef DATABUFFERBASE_H_
#define DATABUFFERBASE_H_

#include <string>

#include "base/lib/xdr/xdr_iarchive.hpp"

#include "BufferBase.h"

/**
 * Base class for input data buffer and its proxy
 */
class InputBufferBase : public BufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

protected:
	//! store new data
	virtual void Store(xdr_iarchive<> & ia) = 0;

public:
	//! Constructor
	InputBufferBase(const std::string & _name);

	//! This is required to make a class polimorphic
	virtual ~InputBufferBase();
};

#endif /* DATABUFFERBASE_H_ */
