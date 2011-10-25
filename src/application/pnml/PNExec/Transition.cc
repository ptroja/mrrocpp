/*
 * Transition.cc
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#include "Transition.hh"

namespace pnexec {

Transition::Transition(NodeId _id, std::string _name, unsigned int _priority)
	: PlaceTransition(_id, _name), priority(_priority)
{
}

Transition::Transition(const transition_t & t)
	: PlaceTransition(t.id(), t.name().value()), priority(t.priority().value())
{
}

} // namespace
