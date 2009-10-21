/*
 * Arc.hh
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#ifndef ARC_HH_
#define ARC_HH_

#include "Node.hh"

#include "pipepnml.hxx"

namespace pnexec {

class Arc: public Node
{
	public:
		const NodeId source, target;

		Arc(NodeId _id, NodeId _source, NodeId _target);

		Arc(const arc_t & a);

//		~Arc() {
//			std::cout << "~Arc(" << id << ")" << std::endl;
//		};
};

} // namespace

#endif /* ARC_HH_ */
