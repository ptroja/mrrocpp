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

#include "planner.h"
#include "plan.hxx"

planner::planner(const std::string & path) :
	state(STOPPED)
{
	// Read plan from XML file
	p = plan(path, xml_schema::Flags::dont_validate);

	// Start operation
	worker = boost::thread(boost::bind(&planner::operator(), this));
}

planner::~planner()
{
	worker.join();
}

const Plan * planner::getPlan(void) const
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
