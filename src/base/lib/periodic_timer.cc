/*
 * periodic_timer.cc
 *
 *  Created on: Jan 22, 2011
 *      Author: ptroja
 */

#include "config.h"

#include <stdexcept>

#if defined(HAVE_POSIX_TIMERS)
	#include <time.h>
	#include <cstring>
	#include <cstdio>
#elif defined(HAVE_KQUEUE)
	#include <sys/types.h>
	#include <sys/event.h>
	#include <sys/time.h>
	#include <cstdio>
#else
	#include <boost/thread/thread.hpp>
	#include <boost/thread/thread_time.hpp>
#endif

#include "base/lib/periodic_timer.h"

namespace mrrocpp {
namespace lib {

#if defined(HAVE_POSIX_TIMERS)
void periodic_timer::timespec_increment_ns(struct timespec * ts, unsigned long increment)
{
	ts->tv_nsec += increment;
	while (ts->tv_nsec >= 1000000000) {
		  ts->tv_sec  += 1;
		  ts->tv_nsec -= 1000000000;
	}
}
#endif

periodic_timer::periodic_timer(unsigned int ms) :
	period_ms(ms)
{
#if defined(HAVE_POSIX_TIMERS)
	if (clock_gettime(CLOCK_MONOTONIC, &wake_time) == -1) {
		perror("clock_gettime()");
		throw std::runtime_error("clock_gettime()");
	}
#elif defined(HAVE_KQUEUE)
	/* create a new kernel event queue */
	if ((kq = kqueue()) == -1) {
		perror("kqueue()");
		throw std::runtime_error("kqueue()");
	}

	/* initalise kevent structure */
	EV_SET(&change, 1, EVFILT_TIMER, EV_ADD | EV_ENABLE, 0, period_ms, 0);
#else
	wakeup = boost::get_system_time();
#endif
}

#if defined(HAVE_KQUEUE)
periodic_timer::~periodic_timer()
{
	close(kq);
}
#endif

void periodic_timer::sleep()
{
#if defined(HAVE_POSIX_TIMERS)
	// Increment the timer
	timespec_increment_ns(&wake_time, period_ms*1000000);

	// @todo loop using the last argument
	int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
	if(err != 0) {
		fprintf(stderr, "clock_nanosleep(): %s\n", strerror(err));
	}
#elif defined(HAVE_KQUEUE)
    /* event that was triggered */
    struct kevent event;

    /* wait for new event */
    int nev = kevent(kq, &change, 1, &event, 1, NULL);

    if (nev < 0) {
            perror("kevent()");
            throw std::runtime_error("kevent()");
    }
    else if (nev > 0) {
            if (event.flags & EV_ERROR) {   /* report any error */
                    fprintf(stderr, "EV_ERROR: %s\n", strerror(event.data));
                    throw std::runtime_error("kevent()");
            }
    }
    else {
            fprintf(stderr, "kevent(): unexpected timeout\n");
            throw std::runtime_error("kevent(): unexpected timeout");
    }
#else
	// Increment the timer
	wakeup += boost::posix_time::millisec(period_ms);

	// Sleep for a while
	boost::thread::sleep(wakeup);
#endif
}

} // namespace lib
} // namespace mrrocpp
