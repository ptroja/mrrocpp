/*
 * DatBufferBase.cc
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#include "InputBufferBase.h"
#include "Agent.h"

InputBufferBase::InputBufferBase(Agent & _owner, const std::string & _name)
	: BufferBase(_name), owner(_owner)
{
	owner.registerBuffer(*this);
}

InputBufferBase::~InputBufferBase()
{
	owner.unregisterBuffer(*this);
}
