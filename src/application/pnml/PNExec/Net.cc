/*
 * Net.cc
 *
 *  Created on: Apr 22, 2009
 *      Author: ptroja
 */

using namespace std;

#include "Net.hh"

#include "pipepnml.hxx"
#include <boost/foreach.hpp>

namespace pnexec {

void Net::Add(Place * _node) {
	// scoped access lock
	boost::mutex::scoped_lock l(mtx);

	std::auto_ptr<Place> ptr(_node);

	places.insert(_node->id, ptr);
}

void Net::Add(Transition * _node) {
	// scoped access lock
	boost::mutex::scoped_lock l(mtx);

	std::auto_ptr<Transition> ptr(_node);

	transitions.insert(_node->id, ptr);
}

void Net::Add(Arc * _arc) {
	// scoped access lock
	boost::mutex::scoped_lock l(mtx);

	std::auto_ptr<Arc> ptr(_arc);

	arcs.insert(_arc->id, ptr);

	places_t::iterator placeIterator;
	transitions_t::iterator transitionIterator;

	if ((placeIterator = places.find(_arc->source)) != places.end()
		&& ((transitionIterator = transitions.find(_arc->target)) != transitions.end())) {
		// connects place -> transition
		Place &source = *(placeIterator->second);
		source.AddOutgoingArc(*_arc);

		Transition &target = *(transitionIterator->second);
		target.AddIncomingArc(*_arc);
	} else if ((placeIterator = places.find(_arc->target)) != places.end()
			&& (transitionIterator = transitions.find(_arc->source)) != transitions.end()) {
		// connects transition -> place
		Transition &source = *(transitionIterator->second);
		source.AddOutgoingArc(*_arc);

		Place &target = *(placeIterator->second);
		target.AddIncomingArc(*_arc);
	}
}

bool Net::ExecuteStep(mrrocpp::mp::common::robots_t & _robots)
{
	// scoped access lock
	boost::mutex::scoped_lock l(mtx);

	// place a mark in Places where robot has finished a job
	BOOST_FOREACH(mrrocpp::mp::common::robot_pair_t & robot_node, _robots) {
		if (robot_node.second->new_pulse && robot_node.second->pulse_code == ECP_WAIT_FOR_NEXT_STATE) {
//			robot_node.second->new_pulse = false;
			Place * p = workers[robot_node.second->robot_name];
			if (p){
				p->addMarker();
				workers.erase(robot_node.second->robot_name);
			}
		}
	}

	// look for active transitions, sort by priority
	std::multimap<unsigned int, Transition*> activeTransitions;

	BOOST_FOREACH(const Transition_pair_t & Transition_node, transitions) {
		Transition &t = *(Transition_node.second);

//		std::cout << "checking transition " << t.name << "...";

		if (!t.IncomingArcs.empty()) {

			bool allPlacesWithMark = true;

			// check for marking
			BOOST_FOREACH(const Arc_pair_t & Arc_node, t.IncomingArcs) {
				// get the Arc
				const Arc &a = *(Arc_node.second);

				// get source Place
				places_t::const_iterator pit = places.find(a.source);
				const Place &p = *(pit->second);

				if(p.getMarking() > 0) {
					continue;
				} else {
					allPlacesWithMark = false;
					break;
				}
			}

			if (allPlacesWithMark) {
//				cout << "active" << endl;
				activeTransitions.insert(std::pair<const unsigned int, Transition*>(t.priority, &t));
			} else {
//				cout << "inactive" << endl;
			}
		}
	}

	if (activeTransitions.empty()) {
//		while(!stop)
//		cond.wait(l);
//		printf("done\n");
		if(workers.size() == 0) {
			fprintf(stderr, "deadlock :-(\n");
		}
		return workers.size();
//		throw DeadlockException("deadlock");
	}

	// print active transitions
//	cout << "Active transitions " << activeTransitions.size() << ":";
//	for(std::multimap<const unsigned int, Transition*>::iterator it = activeTransitions.begin();
//		it != activeTransitions.end(); it++) {
//		cout << " " << (*it->second).name;
//	}
//	cout << endl;

	Transition &execTransition = (*activeTransitions.begin()->second);

	// remove incoming markers
	BOOST_FOREACH(const Arc_pair_t & Arc_node, execTransition.IncomingArcs) {
		// get the Arc
		const Arc &a = *(Arc_node.second);

		// get source Place
		places_t::iterator pit = places.find(a.source);
		Place & p = *(pit->second);

		p.removeMarker();
	}

	// execute highest priority Transition
	execTransition.Execute();

	// place outcoming markers
	BOOST_FOREACH(const Arc_pair_t & Arc_node, execTransition.OutgoingArcs) {
		// get the Arc
		const Arc &a = *(Arc_node.second);

		// get target Place
		places_t::iterator pit = places.find(a.target);
		Place &p = *(pit->second);

		// make this do something usefull
//		executor.create_thread(boost::bind(&PlaceExecutor::execute, &p, &cond, &mtx));
		p.execute(_robots, workers);
	}

	return false;
}

void Net::PrintMarkedPlaces(void) {

	// scoped access lock
	boost::mutex::scoped_lock l(mtx);

	cout << "MarkedPlaces>";
	BOOST_FOREACH(const Place_pair_t & p, places)
	{
		if (1 || p.second->getMarking()) {
			cout << " " << p.second->name << ":" << p.second->getMarking();
		}
	}
	cout << endl;
}

void Net::BuildFromPNML(const char *pnmlfile) {
	// for info about the possible validation flags see
	// http://www.codesynthesis.com/projects/xsd/documentation/cxx/tree/guide/#5.1

	// validate the document before building the tree
	xml_schema::properties props;
	props.no_namespace_schema_location (PNML_SCHEMA_XSD);
	props.schema_location ("http://www.w3.org/XML/1998/namespace", "xml.xsd");

	// bind to the root of PNML file
	auto_ptr<pnml_t> root(pnml(pnmlfile, 0, props));

	net_t &n = root->net();

	// add Places
	BOOST_FOREACH(const place_t &p, n.place()) {
		this->Add(new Place(p));
	}

	// add Transitions
	BOOST_FOREACH(const transition_t & t, n.transition()) {
		this->Add(new Transition(t));
	}

	// add Arcs
	BOOST_FOREACH(const arc_t & a, n.arc()) {
		this->Add(new Arc(a));
	}
}

Net::Net(void) {
}

Net::~Net() {
	// finish the executed tasks
	printf("~Net::wait()..."); fflush(stdout);
	executor.join_all();
	printf("got\n");
}

} // namespace
