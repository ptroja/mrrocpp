#include "Node.hh"

#include <iostream>

namespace pnexec {

Node::Node(NodeId _id) : id(_id) {
//	std::cout << id << std::endl;
}

Node::~Node() {
//	std::cout << "~" << id << std::endl;
}

} // namespace
