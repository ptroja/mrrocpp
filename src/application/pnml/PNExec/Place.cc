/*
 * Place.cc
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#include <iostream>
#include <boost/foreach.hpp>

#include "Place.hh"
#include "base/lib/impconst.h"

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

void Place::execute(mrrocpp::mp::common::robots_t & _robots, workers_t & _workers)
{
	std::cout << "Run " << name << std::endl;

	bool found_a_worker = false;
	BOOST_FOREACH(PNExecToolSpecific & ts, toolspecifics) {
		BOOST_FOREACH(Task & t, ts.tasks) {
			mrrocpp::lib::robot_name_t who = t.execute(_robots, _workers);
			if (who != mrrocpp::lib::ROBOT_UNDEFINED) {
				_workers[who] = this;
				found_a_worker = true;
			}
		}
	}

	// if nothing to do just place a marker in Place
	if(!found_a_worker) addMarker();
}

std::ostream& operator<<(std::ostream &out, Place &cPlace)
{
	// Since operator<< is a friend of the Place class, we can access
	// Place members directly.
	out << cPlace.name << ":" << cPlace.marking;
	return out;
}

void PlaceExecutor::execute(Place *_place, boost::condition_variable *_cond, boost::mutex *_mtx) {
//	_place->execute();
	std::cout << "lock...()" << std::endl;
	boost::mutex::scoped_lock l(*_mtx);
	_place->addMarker();
	_cond->notify_one();
	std::cout << "notify_one...() " << std::endl;
}

} // namespace
