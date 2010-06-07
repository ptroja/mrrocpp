/*
 * RemoteBuffer.h
 *
 *  Created on: May 25, 2010
 *      Author: ptroja
 */

#ifndef REMOTEBUFFER_H_
#define REMOTEBUFFER_H_

#include <boost/serialization/string.hpp>
#include "lib/xdr/xdr_oarchive.hpp"
#include "RemoteAgent.h"

template <class T>
class RemoteBuffer {
private:
	//! name of the buffer
	const std::string name;

	//! owner of the buffer
	RemoteAgent &owner;

public:
	RemoteBuffer(RemoteAgent & _owner, const std::string & _name)
		: name(_name), owner(_owner)
	{
	}

	void Set(const T & data) {
		xdr_oarchive<4096> oa;
		oa << name;
		oa << data;

		owner.Send(oa);
	}
};

#endif /* REMOTEBUFFER_H_ */
