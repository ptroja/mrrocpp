/*
 * BufferBase.cc
 *
 *  Created on: Nov 16, 2011
 *      Author: ptroja
 */

#include <string>

#include "BufferBase.h"

namespace mrrocpp {
namespace lib {
namespace agent {

BufferBase::BufferBase(const std::string & _name)
	: name(_name)
{
}

const std::string & BufferBase::getName() const
{
	return this->name;
}

} // namespace agent
} // namespace lib
} // namespace mrrocpp
