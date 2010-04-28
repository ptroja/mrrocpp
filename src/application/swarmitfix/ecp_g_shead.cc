/*
 * generator/ecp_g_sleep.cc
 *
 *Author: Tomasz Bem
 */

#include "ecp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {


//constructor with parameters: task and time to sleep [s]
head_soldify::head_soldify (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void head_soldify::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool head_soldify::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool head_soldify::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}





















//constructor with parameters: task and time to sleep [s]
head_desoldify::head_desoldify (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void head_desoldify::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool head_desoldify::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool head_desoldify::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}

























//constructor with parameters: task and time to sleep [s]
head_vacuum_on::head_vacuum_on (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void head_vacuum_on::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool head_vacuum_on::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool head_vacuum_on::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}

















//constructor with parameters: task and time to sleep [s]
head_vacuum_off::head_vacuum_off (common::task::task& _ecp_task, double s): generator (_ecp_task){
	if (the_robot) the_robot->communicate_with_edp=false;	//do not communicate with edp
	waittime=s*1000;			//wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec=20000000;	//sleep time[ns]
	sleeptime.tv_sec=0;
}

//allow for later change of a sleep time
void head_vacuum_off::init_time(double s){
	waittime=s*1000; //TODO: conversion from seconds to nanoseconds (?!)
}

bool head_vacuum_off::first_step(){
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){	//acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime=acttime;
	return true;
}

bool head_vacuum_off::next_step(){
	double diff;

	prevtime=acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff=(acttime.tv_sec-prevtime.tv_sec)*1000+(acttime.tv_nsec-prevtime.tv_nsec)/1000000;
	if(diff>100)
		waittime=waittime+diff;

	//difference between start time and actual time, check if wait time already passed
	diff=(acttime.tv_sec-starttime.tv_sec)*1000+(acttime.tv_nsec-starttime.tv_nsec)/1000000;
	if(diff>waittime)
		return false;
	else{
		nanosleep(&sleeptime,NULL);
		return true;
	}
}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

