/*
 * Place.cc
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#include "Place.hh"

#include <iostream>
#include <boost/foreach.hpp>

namespace pnexec {

Place::Place(NodeId _id, std::string _name, unsigned int _initial_marking, int _capacity)
	: PlaceTransition(_id, _name), marking(_initial_marking), capacity(_capacity) {
}

Place::Place(const place_t & p)
	: PlaceTransition(p.id(), p.name().value()),
	marking(p.initialMarking().value()), capacity(p.capacity().value())
{
	// Iteration.
	BOOST_FOREACH(const toolspecific_t & toolspec, p.toolspecific()) {
		toolspecifics.push_back(new PNExecToolSpecific (toolspec));
	}
}

void Place::run(void){
	std::cout << "Run " << name << std::endl;
	BOOST_FOREACH(PNExecToolSpecific & ts, toolspecifics) {
		BOOST_FOREACH(Task & t, ts.tasks) {
			t.execute();
		}
	}
}

std::ostream& operator<<(std::ostream &out, Place &cPlace)
{
	// Since operator<< is a friend of the Place class, we can access
	// Place members directly.
	out << cPlace.name << ":" << cPlace.marking;
	return out;
}

void PlaceExecutor::run(Place *_place, boost::condition_variable *_cond, boost::mutex *_mtx) {
	_place->run();
	std::cout << "lock...()" << std::endl;
	boost::mutex::scoped_lock l(*_mtx);
	_place->addMarker();
	_cond->notify_one();
	std::cout << "notify_one...() " << std::endl;
}

} // namespace
