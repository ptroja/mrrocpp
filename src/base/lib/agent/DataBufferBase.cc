/*
 * DatBufferBase.cc
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#include "DataBufferBase.h"

DataBufferBase::DataBufferBase(const std::string & _name)
	: name(_name)
{
}

const std::string & DataBufferBase::getName() const
{
	return name;
}

DataBufferBase::~DataBufferBase()
{
}
