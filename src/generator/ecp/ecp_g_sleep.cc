/*
 * generator/ecp_g_sleep.cc
 *
 *Author: Tomasz Bem
 */

#include <cstdio>

#include "base/ecp/ecp_robot.h"
#include "generator/ecp/ecp_g_sleep.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

//constructor with parameters: task and time to sleep [s]
sleep::sleep(common::task::task& _ecp_task, double s) :
	generator(_ecp_task)
{
	if (the_robot)
		the_robot->communicate_with_edp = false; //do not communicate with edp
	waittime = s * 1000; //wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec = 20000000; //sleep time[ns]
	sleeptime.tv_sec = 0;
}

//allow for later change of a sleep time
void sleep::init_time(double s)
{
	waittime = s * 1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool sleep::first_step()
{
	if (clock_gettime(CLOCK_REALTIME, &acttime) == -1) { //acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime = acttime;
	return true;
}

bool sleep::next_step()
{
	double diff;

	prevtime = acttime;
	if (clock_gettime(CLOCK_REALTIME, &acttime) == -1) {
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff = (acttime.tv_sec - prevtime.tv_sec) * 1000 + (acttime.tv_nsec - prevtime.tv_nsec) / 1000000;
	if (diff > 100)
		waittime = waittime + diff;

	//difference between start time and actual time, check if wait time already passed
	diff = (acttime.tv_sec - starttime.tv_sec) * 1000 + (acttime.tv_nsec - starttime.tv_nsec) / 1000000;
	if (diff > waittime)
		return false;
	else {
		nanosleep(&sleeptime, NULL);
		return true;
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

