/*
 * BufferBase.cc
 *
 *  Created on: Nov 16, 2011
 *      Author: ptroja
 */

#include <string>

#include "BufferBase.h"

BufferBase::BufferBase(const std::string & _name)
	: name(_name)
{
}

const std::string & BufferBase::getName() const
{
	return this->name;
}
