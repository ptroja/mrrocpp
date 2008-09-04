/*
 * ecp_g_sleep.cc
 *
 *  Created on: Sep 3, 2008
 *      Author: ghard
 */

#include "ecp/common/ecp_g_sleep.h"
#include <iostream>

ecp_sleep_generator::ecp_sleep_generator (ecp_task& _ecp_task, int s)
: ecp_generator (_ecp_task)
{
	seconds = s;
    current_time = (int)time((time_t*)NULL);
    wait_time = (int)time((time_t*)NULL);
}

int ecp_sleep_generator::init_time(int s)
{
	seconds = s;
	wait_time = (int)time((time_t*)NULL) + s;
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
	wait_time = (int)time((time_t*)NULL) + seconds;
    return true;
}

bool ecp_sleep_generator::next_step()
{
	current_time = (int)time((time_t*)NULL);
//std::cout << current_time << "        " << wait_time << std::endl;
	if(current_time<wait_time)
		return true;
	else
		return false;
}


