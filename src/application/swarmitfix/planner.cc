/*
 * planner.cc
 *
 *  Created on: Nov 23, 2011
 *      Author: ptroja
 */

#include <string>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/bind.hpp>

#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#include "planner.h"
#include "plan.hxx"

std::string planner::planpath = "planpath";

planner::planner(const std::string & path) :
	state(STOPPED)
{
	//throw std::runtime_error("test");

	// Assume, that the XSD file is installed in the binary folder
	boost::filesystem::path xsdpath = boost::filesystem::current_path();
	xsdpath /= "plan.xsd";

	// XML validation settings
	xml_schema::Properties props;

	// Add XSD validation to parser's properties
#if BOOST_VERSION >=104400
	props.no_namespace_schema_location (xsdpath.string());
#else
	props.no_namespace_schema_location (xsdpath.file_string());
#endif

	// Read plan from XML file
	try {
		// If we had no XML schema, then only limited validation is possible:
		// p = plan(path, xml_schema::Flags::dont_validate);

		// Parse file with all the schema checks
		p = plan(path, 0, props);
	} catch (const xml_schema::Exception & e) {
		// Display detailed diagnostics
		std::cerr << e << std::endl;

		// And leave the rest to the high-level handler
		throw;
	}

	// Start triggering operation
	worker = boost::thread(boost::bind(&planner::operator(), this));
}

planner::~planner()
{
	worker.interrupt();
	worker.join();
}

Plan * planner::getPlan(void) const
{
	return p.get();
}

void planner::start()
{
	// Lock access to the state variable
	boost::mutex::scoped_lock lock(mtx);

	if (state != STOPPED) {
		// TODO: throw
	}

	state = RUNNING;
}


void planner::pause()
{
	// Lock access to the state variable
	boost::mutex::scoped_lock lock(mtx);

	if (state != RUNNING) {
		// TODO: throw
	}

	state = PAUSED;
}

void planner::resume()
{
	// Lock access to the state variable
	boost::mutex::scoped_lock lock(mtx);

	if (state != PAUSED) {
		// TODO: throw
	}

	state = RUNNING;
}

void planner::operator()()
{
	std::cerr << "planner task started" << std::endl;
	// std::cerr << "pNum " << p->pNum() << std::endl;

	// Never-ending planner operation
	do
	{
		{
			// Lock access to the state variable
			boost::mutex::scoped_lock lock(mtx);

			// Wait until operation is started
			while(state != RUNNING) {
				cond.wait(lock);
			}
		}

		// Start of the plan execution
		const boost::system_time startup = boost::get_system_time();

		// Next trigger time
		boost::system_time next_trigger;

		for(Plan::PkmType::ItemConstIterator it = p->pkm().item().begin();
				it != p->pkm().item().end();
				++it) {
			const Plan::PkmType::ItemType & pCmd = *it;

			//std::cerr << "[" << pCmd.l1() << "," << pCmd.l2() << "," << pCmd.l3() << "]" << std::endl;

			next_trigger = startup + boost::posix_time::seconds(pCmd.TBeg());

			// Sleep until next triggering
			boost::thread::sleep(next_trigger);
		}
	} while(0);

	std::cerr << "planner task finished" << std::endl;
}
