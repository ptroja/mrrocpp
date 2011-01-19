/*!
 * @file timer.h
 * @brief Utility timer class	.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <cstdio>
#include <sys/time.h>

#include "base/lib/timer.h"

namespace mrrocpp {
namespace lib {

timer::timer() :
	initialized(true), started(false), stopped(false), last_status(TIMER_INITIALIZED)
{
}

timer::timer_status_t timer::start()
{
	if (initialized) {
		if (gettimeofday(&t1, NULL) == -1) {
			perror("gettimeofday()");
		}
		started = true;
		stopped = false;
		last_status = TIMER_STARTED;
		return TIMER_STARTED;
	} else {
		last_status = TIMER_NOT_INITIALIZED;
		return TIMER_NOT_INITIALIZED;
	}
}

timer::timer_status_t timer::stop()
{
	if (initialized) {
		if (started) {
			if (gettimeofday(&t2, NULL) == -1) {
				perror("gettimeofday()");
			}
			// timer_started = false; // by Y
			stopped = true;
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

timer::timer_status_t timer::get_time(float & sec)
{
	if (stopped) {
		sec = (t2.tv_sec + t2.tv_usec / 1e6) - (t1.tv_sec + t1.tv_usec / 1e6);
		last_status = TIME_RETRIVED;
		return TIME_RETRIVED;
	} else {
		last_status = TIMER_RUNNING_OR_NOT_STARTED;
		return TIMER_RUNNING_OR_NOT_STARTED;
	}
}

void timer::print_last_status() const
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
		default:
			printf("TIMER_UNKNOWN_STATUS\n");
			break;
	}
}

} // namespace lib
} // namespace mrrocpp

