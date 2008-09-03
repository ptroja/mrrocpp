/*
 * ecp_g_sleep.cc
 *
 *  Created on: Sep 3, 2008
 *      Author: ghard
 */

#include "ecp/common/ecp_g_sleep.h"

ecp_sleep_generator::ecp_sleep_generator (ecp_task& _ecp_task, int ms)
: ecp_generator (_ecp_task)
{
	miliseconds = ms;
    current_time = (int)time((time_t*)NULL);
    wait_time = (int)time((time_t*)NULL);
}

int ecp_sleep_generator::init_time(int ms)
{
	miliseconds = ms;
	wait_time = (int)time((time_t*)NULL) + ms;
}

bool ecp_sleep_generator::is_now()
{
    current_time = (int)time((time_t*)NULL);
	if(current_time>=wait_time)
		return true;
	else
		return false;
}
bool ecp_sleep_generator::first_step()
{
	wait_time = (int)time((time_t*)NULL) + miliseconds;
    return true;
}

bool ecp_sleep_generator::next_step()
{
	current_time = (int)time((time_t*)NULL);
	if(current_time<wait_time)
		return true;
	else
		return false;
}


