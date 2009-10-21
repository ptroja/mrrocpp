/*
 * PlaceTransition.cc
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#include "PlaceTransition.hh"

namespace pnexec {

PlaceTransition::PlaceTransition(NodeId _id, std::string _name)
	: Node(_id), name(_name) {
}

PlaceTransition::~PlaceTransition() {
}

void PlaceTransition::AddIncomingArc(Arc &_arc) {
	IncomingArcs[_arc.id] = &_arc;
}

void PlaceTransition::AddOutgoingArc(Arc &_arc) {
	OutgoingArcs[_arc.id] = &_arc;
}

} // namespace
