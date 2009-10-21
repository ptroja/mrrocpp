/*
 * PlaceTransition.hh
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#ifndef PLACETRANSITION_HH_
#define PLACETRANSITION_HH_

#include "Node.hh"
#include "Arc.hh"
#include "PNExecToolSpecific.hh"

#include <boost/ptr_container/ptr_list.hpp>

namespace pnexec {

class PlaceTransition : public Node {

	public:
		std::map<NodeId, Arc *> OutgoingArcs;
		std::map<NodeId, Arc *> IncomingArcs;

		const std::string name;

	protected:

		boost::ptr_list<PNExecToolSpecific> toolspecifics;

	public:
		void AddOutgoingArc(Arc &_arc);
		void AddIncomingArc(Arc &_arc);
		PlaceTransition(NodeId _id, std::string _name = std::string(""));
		virtual ~PlaceTransition();
};

} // namespace

#endif /* PLACETRANSITION_HH_ */
