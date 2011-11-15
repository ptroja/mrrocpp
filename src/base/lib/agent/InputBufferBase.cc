/*
 * DatBufferBase.cc
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#include "InputBufferBase.h"

InputBufferBase::InputBufferBase(const std::string & _name)
	: name(_name)
{
}

const std::string & InputBufferBase::getName() const
{
	return name;
}

InputBufferBase::~InputBufferBase()
{
}
