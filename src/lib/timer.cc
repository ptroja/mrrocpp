#include <cstdio>
#include <time.h>

#include "lib/timer.h"

namespace mrrocpp {
namespace lib {

timer::timer(void)
{
	timer_initialized = true;
	timer_stopped = false;
	timer_started = false;
	last_status = TIMER_INITIALIZED;
}

timer::timer_status_t timer::timer_start(void)
{
	if (timer_initialized) {
		if (clock_gettime(CLOCK_REALTIME, &t1) == -1) {
			perror("clock_gettime()");
		}
		timer_started = true;
		timer_stopped = false;
		last_status = TIMER_STARTED;
		return TIMER_STARTED;
	} else {
		last_status = TIMER_NOT_INITIALIZED;
		return TIMER_NOT_INITIALIZED;
	}
}

timer::timer_status_t timer::timer_stop(void)
{
	if (timer_initialized) {
		if (timer_started) {
			if (clock_gettime(CLOCK_REALTIME, &t2) == -1) {
				perror("clock_gettime()");
			}
			// timer_started = false; // by Y
			timer_stopped = true;
			// timer_initialized = false; // by Y
			last_status = TIMER_STOPPED;
			return TIMER_STOPPED;
		} else {
			last_status = TIMER_NOT_STARTED;
			return TIMER_NOT_STARTED;
		}
	} else {
		last_status = TIMER_NOT_INITIALIZED;
		return TIMER_NOT_INITIALIZED;
	}
}

timer::timer_status_t timer::get_time(float *sec)
{
	if (timer_stopped) {
		float t = (t2.tv_sec + t2.tv_nsec / 1e9) - (t1.tv_sec + t1.tv_nsec / 1e9);
		if (sec)
			*sec = t;
		last_status = TIME_RETRIVED;
		return TIME_RETRIVED;
	} else {
		last_status = TIMER_RUNNING_OR_NOT_STARTED;
		return TIMER_RUNNING_OR_NOT_STARTED;
	}
}

void timer::print_last_status(void)
{
	switch (last_status)
	{
		case TIMER_INITIALIZED:
			printf("TIMER_INITIALIZED\n");
			break;
		case TIMER_STARTED:
			printf("TIMER_STARTED\n");
			break;
		case TIMER_STOPPED:
			printf("TIMER_STOPPED\n");
			break;
		case TIME_RETRIVED:
			printf("TIME_RETRIVED\n");
			break;
		case CYCLES_RETRIVED:
			printf("CYCLES_RETRIVED\n");
			break;
		case TIMER_NOT_INITIALIZED:
			printf("TIMER_NOT_INITIALIZED\n");
			break;
		case TIMER_NOT_STARTED:
			printf("TIMER_NOT_STARTED\n");
			break;
		case TIMER_NOT_STOPPED:
			printf("TIMER_NOT_STOPPED\n");
			break;
		case TIMER_RUNNING_OR_NOT_STARTED:
			printf("TIMER_RUNNING_OR_NOT_STARTED\n");
			break;
	}
}

} // namespace lib
} // namespace mrrocpp

