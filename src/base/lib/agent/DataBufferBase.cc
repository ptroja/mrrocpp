/*
 * DatBufferBase.cc
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#include "DataBufferBase.h"
#include "AndDataCondition.h"
#include "OrDataCondition.h"

DataBufferBase::DataBufferBase(const std::string & _name)
	: name(_name), new_data_ready(false)
{
}

const std::string & DataBufferBase::getName() const
{
	return name;
}

bool DataBufferBase::isNewData() const
{
	return new_data_ready;
}

AndDataCondition DataBufferBase::operator&(DataBufferBase &op) {
	AndDataCondition container(*this);
	container.push_back(&op);

	return container;
}

AndDataCondition DataBufferBase::operator&(AndDataCondition &op) {
	AndDataCondition container(op);
	container.push_back(this);

	return container;
}

OrDataCondition DataBufferBase::operator|(DataBufferBase &op) {
	AndDataCondition c1(*this);
	AndDataCondition c2(op);

	OrDataCondition c = c1 | c2;

	return c;
}

OrDataCondition DataBufferBase::operator|(AndDataCondition &op) {
	OrDataCondition c(*this);
	c.push_back(op);

	return c;
}

OrDataCondition DataBufferBase::operator|(OrDataCondition &op) {
	OrDataCondition c1(op);
	AndDataCondition c2(*this);
	c1.push_back(c2);

	return c1;
}

DataBufferBase::~DataBufferBase()
{
}
