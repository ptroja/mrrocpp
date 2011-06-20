/*
 * Arc.cc
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#include "Arc.hh"

namespace pnexec {

Arc::Arc(NodeId _id, NodeId _source, NodeId _target)
	: Node(_id), source(_source), target(_target) {
}

Arc::Arc(const arc_t & a)
	: Node(a.id()), source(a.source()), target(a.target()) {
}

} // namespace
