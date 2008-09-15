/*
 * ecp_g_sleep.cc
 *
 *Author: Tomasz Bem
 */

#include "ecp/common/ecp_g_sleep.h"

//constructor with parameters: task and time to sleep [s]
ecp_sleep_generator::ecp_sleep_generator (ecp_task& _ecp_task, int s): ecp_generator (_ecp_task){
	communicate_with_edp=false;	//do not communicate with edp
	seconds=s;							//wait time[s]
	wait.tv_nsec=20000000;			//sleep part time[ns]
}

//allow for later change of a sleep time
int ecp_sleep_generator::init_time(int s){
	seconds=s;
}

bool ecp_sleep_generator::first_step(){
	remain=seconds*1000/2.7;			//wai time in [ms] divided by correction factor it may be changed, 
	return true;						//just take stopwatch and if given wait time is not correct chagne it
}

bool ecp_sleep_generator::next_step(){
	
	if(remain>0){							//case in which wait time did not pass
		remain-=20;						//subtract sleep time
		nanosleep(&wait,NULL);		//in nano seconds
		return true;					//generate next step
	}else
		return false;					//stop generator
}


