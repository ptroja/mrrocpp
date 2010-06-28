/*
 * PathPlan.cc
 *
 *  Created on: Jun 24, 2010
 *      Author: ptroja
 */

#include <boost/foreach.hpp>
#include <memory>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "PathPlan.h"

#include "swarmplan.hxx"

void PathPlan::load(const std::string & filename)
{
	// Create an empty property tree object
	using boost::property_tree::ptree;
	ptree pt;

	read_xml(filename, pt);

	write_xml("output.xml", pt);
	write_json("output.json", pt);

	return;

	// for info about the possible validation flags see
	// http://www.codesynthesis.com/projects/xsd/documentation/cxx/tree/guide/#5.1

	// validate the document before building the tree
	::xml_schema::properties props;
	props.no_namespace_schema_location ("swarmplan.xsd");
	props.schema_location ("http://www.w3.org/XML/1998/namespace", "xml.xsd");

	// bind to the root of PNML file
	std::auto_ptr<PathPlan_t> root(pathPlan(filename, 0, props));

	PathPlan_t::agents_type & a = root->agents();

	std::cout << "a.plan().size() = " << a.plan().size() << std::endl;

#if 0
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
#endif
}

int
main(int argc, char *argv[])
{
	PathPlan pp;

	pp.load("instance1.xml");

	return 0;
}
