/*
 * periodic_timer.h
 *
 *  Created on: Jan 22, 2011
 *      Author: ptroja
 */

#ifndef PERIODIC_TIMER_H_
#define PERIODIC_TIMER_H_

#include "config.h"

#include <boost/utility.hpp>

#if defined(HAVE_POSIX_TIMERS)
	#include <time.h>
#elif defined(HAVE_KQUEUE)
	#include <sys/types.h>
	#include <sys/event.h>
	#include <sys/time.h>
#else
	#include <boost/thread/thread_time.hpp>
#endif

namespace mrrocpp {
namespace lib {

//! Class implementing a high-resolution portable periodic timer
//! @note monotonic time is preferred over a system time
class periodic_timer : boost::noncopyable {
private:
	//! Period in milliseconds
	const int period_ms;
#if defined(HAVE_POSIX_TIMERS)
	//! When to wake up next time
	struct timespec wake_time;

	//! Increment timespec structure with a number of nanoseconds taking care about overflows
	//! @param ts pointer to the timespec structure
	//! @param increment value of the increment in nanoseconds
	//! @note This is a C-style call so we want a pointer instead of a C++ reference
	void timespec_increment_ns(struct timespec * ts, unsigned long increment);
#elif defined(HAVE_KQUEUE)
	//! Kqueue file descriptor
	int kq;

	//! Event we want to monitor
	struct kevent change;
#else
	//! Wakeup time
	boost::posix_time::ptime wakeup;
#endif
public:
	//! Constructor
	//! @param ms period in milliseconds
	//! @note cureently no more precision than a millisecond is required
	periodic_timer(unsigned int ms);

	//! Wait until next firing
	void sleep();

#if defined(HAVE_KQUEUE)
	//! Destructor
	~periodic_timer();
#endif
};

} // namespace lib
} // namespace mrrocpp

#endif /* PERIODIC_TIMER_H_ */
