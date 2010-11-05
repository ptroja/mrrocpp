/*
 * AndDataCondition.cc
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#include <ostream>

#include <boost/foreach.hpp>

#include "AndDataCondition.h"

AndDataCondition::AndDataCondition()
{
}

AndDataCondition::AndDataCondition(const DataBufferBase &op)
{
	this->push_back(&op);
}

bool AndDataCondition::isNewData() const
{
	BOOST_FOREACH(const DataBufferBase * ptr, *this) {
		if (!ptr->isNewData()) {
			return false;
		}
	}
	return true;
}

OrDataCondition AndDataCondition::operator|(const AndDataCondition &op)
{
	OrDataCondition cont(*this);
	cont.push_back(op);

	return cont;
}

OrDataCondition AndDataCondition::operator|(const OrDataCondition &op)
{
	OrDataCondition cont(op);
	cont.push_back(*this);

	return cont;
}

AndDataCondition AndDataCondition::operator&(const AndDataCondition &op)
{
	AndDataCondition cont(*this);
	BOOST_FOREACH(const DataBufferBase * ptr, op) {
		cont.push_back(ptr);
	}
	return cont;
}

AndDataCondition AndDataCondition::operator&(const DataBufferBase &op)
{
	AndDataCondition cont(*this);
	this->push_back(&op);
	return cont;
}

AndDataCondition & AndDataCondition::operator=(const DataBufferBase &op)
{
	this->clear();
	this->push_back(&op);
	return *this;
}

std::ostream& operator<<(std::ostream& output, const AndDataCondition& me)
{
	for(std::size_t i = 0; i < me.size(); ++i) {
		const DataBufferBase * element = me.operator[](i)	;
		output << element->getName();
		if(i < me.size() - 1) {
			output << " & ";
		}
	}

	return output;
}
