/*
 * Node.hh
 *
 *  Created on: Apr 20, 2009
 *      Author: ptroja
 */

#ifndef NODE_HH_
#define NODE_HH_

#include <iostream>
#include <map>

namespace pnexec {

//class NodeId : public std::string {
//
//};

typedef std::string NodeId;

class Node
{
	public:
		const NodeId id;

		Node(NodeId _id);
		virtual ~Node();
};

} // namespace

#endif /* NODE_HH_ */
