/*
 * DatBufferBase.cc
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#include "InputBufferBase.h"
#include "Agent.h"

namespace mrrocpp {
namespace lib {
namespace agent {

InputBufferBase::InputBufferBase(Agent & _owner, const std::string & _name)
	: BufferBase(_name), owner(_owner)
{
	owner.registerBuffer(*this);
}

InputBufferBase::~InputBufferBase()
{
	owner.unregisterBuffer(*this);
}

} // namespace agent
} // namespace lib
} // namespace mrrocpp
