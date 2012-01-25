/*
 * OutputBuffer.h
 *
 *  Created on: Nov 13, 2011
 *      Author: ptroja
 */

#ifndef OUTPUTBUFFER_H_
#define OUTPUTBUFFER_H_

#include <string>

#include "RemoteAgent.h"
#include "BufferBase.h"
#include "base/lib/xdr/xdr_oarchive.hpp"

namespace mrrocpp {
namespace lib {
namespace agent {

/**
 * Remote data buffer proxy
 */
template <class T>
class OutputBuffer : public BufferBase {
private:
	//! Owner of the buffer
	RemoteAgent & owner;

public:
	//! Construct remote buffer proxy
	OutputBuffer(RemoteAgent & _owner, const std::string & _name)
		: BufferBase(_name), owner(_owner)
	{
	}

	//! Set the contents of the remote buffer
	void Send(const T & data) {
		xdr_oarchive<> oa;
		oa << this->getName();
		oa << data;

		owner.Send(oa);
	}
};

} // namespace agent
} // namespace lib
} // namespace mrrocpp

#endif /* OUTPUTBUFFER_H_ */
