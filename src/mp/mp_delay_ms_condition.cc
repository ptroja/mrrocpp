#include "mp/mp_delay_ms_condition.h"

#include "unistd.h"

// condition to wait for desired time in ms

mp_delay_ms_condition::mp_delay_ms_condition(mp_task& _mp_task, int _ms_delay): mp_generator (_mp_task)
{
	local_timer = new mp_timer();
	configure(_ms_delay);
}

mp_delay_ms_condition::~mp_delay_ms_condition()
{
	delete local_timer;
}

void mp_delay_ms_condition::configure (int _ms_delay)
{
	ms_delay = _ms_delay;
}

bool mp_delay_ms_condition::first_step ()
{
	local_timer->timer_start(NULL);
	return true;
}

bool mp_delay_ms_condition::next_step ()
{
	local_timer->timer_stop(NULL);
	local_timer->get_time(&sec);
	if (1000*sec > (float) ms_delay)
		return false;
	delay (20);
	local_timer->timer_stop(NULL);
	local_timer->get_time(&sec);
	if (1000*sec > (float) ms_delay)
		return false;
	return true;
}
