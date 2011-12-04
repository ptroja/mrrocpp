/*
 * planner.h
 *
 *  Created on: Nov 23, 2011
 *      Author: ptroja
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include "plan.hxx"

//! Interface with the planner and/or
class planner {
public:
	//! Constructor
	//! @param path path to the XML file with the plan
	planner(const std::string & path);

	//! Destructor
	~planner();

	//! Start execution of the plan
	void start();

	//! Pause execution of the plan
	void pause();

	//! Resume execution of the plan
	void resume();

	//! Access to the plan data
	const Plan * getPlan(void) const;

private:
	//! Reference to the plan
	boost::shared_ptr<Plan> p;

	//! State of the worker thread
	enum _state { STOPPED, RUNNING, PAUSED} state;

	//! Internal thread for triggering plan progress
	boost::thread worker;

	//! Condition variable for start/stop/pause synchronization
	boost::condition_variable cond;

	//! Mutex for access to start/stop/pause synchronization
	boost::mutex mtx;

	//! Internal worker routine
	void operator()();
};

#endif /* PLANNER_H_ */
