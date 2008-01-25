#include <inttypes.h>
#include <stdio.h>

#if defined(__QNXNTO__)
#include <sys/syspage.h>
#include <sys/neutrino.h>
#else
uint64_t ClockCycles(void);
#endif /* __QNXNTO__ */

#include "lib/mp_timer.h"

mp_timer::mp_timer(void)
{
	timer_initialized = 0;
	timer_started = 0;
	timer_stopped = 0;
#if defined(__QNXNTO__)
	cycles = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
#endif /* __QNXNTO__ */
	timer_initialized = 1;
	timer_stopped = 0;
	timer_started = 0;
	last_status = TIMER_INITIALIZED;

} // timer_init


mp_timer_status_enum mp_timer::timer_start(uint64_t *t)
{
	if (timer_initialized)
	{
		t1 = ClockCycles();
		if (t!=NULL)
			*t = t1;
		timer_started = 1;
		timer_stopped = 0;
		last_status = TIMER_STARTED;
		return TIMER_STARTED;
	} else {
		last_status = TIMER_NOT_INITIALIZED;
		return TIMER_NOT_INITIALIZED;
	}

} // timer_start


mp_timer_status_enum mp_timer::timer_stop(uint64_t *t)
{
	if (timer_initialized)
	{
		if (timer_started)
		{
			t2 = ClockCycles();
			if (t!=NULL)
				*t = t2;
			// timer_started = 0; // by Y
			timer_stopped = 1;
			// timer_initialized = 0; // by Y
			last_status = TIMER_STOPPED;
			return TIMER_STOPPED;
		} else {
			last_status = TIMER_NOT_STARTED;
			return TIMER_NOT_STARTED;
		}
	}
	else {
		last_status = TIMER_NOT_INITIALIZED;
		return TIMER_NOT_INITIALIZED;
	}
		
} // timer_stop


mp_timer_status_enum mp_timer::get_time(float *sec)
{
	if (timer_stopped)
	{
		ncycles = t2 - t1;
		*sec = (float)ncycles / cycles;
		last_status = TIME_RETRIVED;
		return TIME_RETRIVED;
	} else {
		last_status = TIMER_RUNNING_OR_NOT_STARTED;
		return TIMER_RUNNING_OR_NOT_STARTED;
	}
	
} // get_time


mp_timer_status_enum mp_timer::get_cycles(uint64_t *c, uint64_t *nc)
{
	if (timer_stopped)
	{
		if (c!=NULL)
			*c = cycles;
		if (nc!=NULL)
			*nc = ncycles;
		// timer_stopped = 0; // by Y
		last_status = CYCLES_RETRIVED;
		return CYCLES_RETRIVED;
	} else {
		last_status = TIMER_NOT_STOPPED;
		return TIMER_NOT_STOPPED;
	}

} // get_cycles

void mp_timer::print_last_status(void)
{
	switch(last_status)
	{
		case TIMER_INITIALIZED: printf("TIMER_INITIALIZED\n");
		break;
		case TIMER_STARTED: printf("TIMER_STARTED\n");
		break;
		case TIMER_STOPPED: printf("TIMER_STOPPED\n");
		break;
		case TIME_RETRIVED: printf("TIME_RETRIVED\n");
		break;
		case CYCLES_RETRIVED: printf("CYCLES_RETRIVED\n");
		break;
		case TIMER_NOT_INITIALIZED: printf("TIMER_NOT_INITIALIZED\n");
		break;
		case TIMER_NOT_STARTED: printf("TIMER_NOT_STARTED\n");
		break;
		case TIMER_NOT_STOPPED: printf("TIMER_NOT_STOPPED\n");
		break;
		case TIMER_RUNNING_OR_NOT_STARTED: printf("TIMER_RUNNING_OR_NOT_STARTED\n");
		break;
	}
} // print_last_status
