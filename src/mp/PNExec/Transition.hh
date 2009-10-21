/*
 * Transition.hh
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#ifndef TRANSITION_HH_
#define TRANSITION_HH_

#include "Node.hh"
#include "PlaceTransition.hh"
#include "pipepnml.hxx"

namespace pnexec {

class Transition: public PlaceTransition
{
	public:
		Transition(NodeId _id, std::string _name = std::string(""), unsigned int _priority = 1);

		Transition(const transition_t & t);

		const unsigned int priority;

		virtual void Execute(void) {
			std::cout << "Execute " << name << std::endl;
		};

		virtual ~Transition() {};
};

} // namespace

#endif /* TRANSITION_HH_ */
