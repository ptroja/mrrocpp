#include <iostream>

#include <boost/foreach.hpp>

#include "DataBuffer.hh"

AndBufferContainer & AndBufferContainer::operator=(const DataBufferBase &op)
{
	this->clear();
	this->push_back(&op);
	return *this;
}

AndBufferContainer & AndBufferContainer::operator&&(const AndBufferContainer &op)
{
	BOOST_FOREACH(const DataBufferBase * ptr, op) {
		this->push_back(ptr);
	}
	return *this;
}

AndBufferContainer & AndBufferContainer::operator&&(const DataBufferBase &op)
{
	this->push_back(&op);
	return *this;
}

OrBufferContainer AndBufferContainer::operator||(const DataBufferBase &op)
{
	OrBufferContainer cont(op);
	return cont;
}

OrBufferContainer AndBufferContainer::operator||(const AndBufferContainer &op)
{
	OrBufferContainer cont(op);
	return cont;
}

AndBufferContainer::AndBufferContainer(const DataBufferBase &op) : fresh(false)
{
	this->push_back(&op);
}

AndBufferContainer::AndBufferContainer() : fresh(false)
{
}

bool AndBufferContainer::isFresh() const
{
	BOOST_FOREACH(const DataBufferBase * ptr, *this) {
		if (!ptr->isFresh()) {
			return false;
		}
	}
	return true;
}

OrBufferContainer & OrBufferContainer::operator=(const DataBufferBase &op)
{
	this->clear();
	AndBufferContainer cont(op);
	this->push_back(cont);
	return *this;
}

OrBufferContainer::OrBufferContainer(const DataBufferBase &op)
{
	AndBufferContainer cont(op);
	this->push_back(cont);
}

OrBufferContainer::OrBufferContainer(const AndBufferContainer &op)
{
	this->push_back(op);
}

OrBufferContainer::OrBufferContainer()
{
}

DataBufferBase::DataBufferBase(const std::string & _name)
	: name(_name), fresh(false)
{
}

const std::string & DataBufferBase::getName() const
{
	return name;
}

bool DataBufferBase::isFresh() const
{
	return fresh;
}

AndBufferContainer DataBufferBase::operator&&(DataBufferBase &op) {
	AndBufferContainer container(*this);
	container.push_back(&op);

	return container;
}

OrBufferContainer DataBufferBase::operator||(DataBufferBase &op) {
	AndBufferContainer c1(*this);
	AndBufferContainer c2(op);

	OrBufferContainer c = c1 || c2;

	return c;
}

OrBufferContainer DataBufferBase::operator||(AndBufferContainer &op) {
	OrBufferContainer c(*this);
	c.push_back(op);

	return c;
}

//! This is required to make a class polimorphic
DataBufferBase::~DataBufferBase() {};
