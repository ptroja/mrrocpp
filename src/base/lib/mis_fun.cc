/*!
 * @file mis_fun.cc
 * @brief Thread utility functions.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <pthread.h>
#include <cstdio>
#include <unistd.h>

#include "base/lib/mis_fun.h"

#if defined(linux)
#include <sys/prctl.h>
#elif defined(__FreeBSD__)
#include <pthread_np.h>
#endif

namespace mrrocpp {
namespace lib {

//! Set the process scheduler
void set_process_sched()
{

	//	int policy;
	struct sched_param param;

	int policy_priority_min = sched_get_priority_min(SCHED_FIFO);
	if (policy_priority_min == -1) {
		perror("sched_get_priority_min() ");
	}

	param.sched_priority = policy_priority_min;

	if (sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler() ");
	}

}

void set_thread_priority(pthread_t thread, int sched_priority_l)
{
	int policy;
	struct sched_param param;
	if (pthread_getschedparam(thread, &policy, &param)) {
		perror("pthread_getschedparam() ");
	}

	// check priority range
	int policy_priority_min = sched_get_priority_min(SCHED_FIFO);
	if (policy_priority_min == -1) {
		perror("sched_get_priority_min() ");
	}

	int policy_priority_max = sched_get_priority_max(SCHED_FIFO);
	if (policy_priority_max == -1) {
		perror("sched_get_priority_max() ");
	}

	//pthread_setscheduler(thread, SCHED_FIFO, sched_priority_l);


	if ((sched_priority_l < policy_priority_min) || (sched_priority_l > policy_priority_max)) {
		fprintf(stderr, "requested thread priority (%d) not in <%d:%d> priority range\n", sched_priority_l, policy_priority_min, policy_priority_max);
	} else {
		param.sched_priority = sched_priority_l;
		if (pthread_setschedparam(thread, SCHED_FIFO, &param)) {
			perror("pthread_setschedparam() ");
		}
	}
}

//! set the thread name for debugging
int set_thread_name(const char * newname)
{
#if defined(linux)
	char comm[16];
	snprintf(comm, sizeof(comm), "%s", newname);
	return prctl(PR_SET_NAME, comm, 0l, 0l, 0l);
#elif defined(__FreeBSD__)
	// FreeBSD defines this call to void
	pthread_set_name_np(pthread_self(), newname);

	return 0;
#endif
	return -1;
}

#ifndef TIMESPEC_VALID
# define TIMESPEC_VALID(__ts) ((__ts)->tv_nsec >= 0 && (__ts)->tv_nsec < 1000000000L)
#endif

void timespec_increment_ns(struct timespec * ts, unsigned long increment)
{
	ts->tv_nsec += increment;
	while (ts->tv_nsec >= 1000000000) {
		ts->tv_sec += 1;
		ts->tv_nsec -= 1000000000;
	}
}

} // namespace lib
} // namespace mrrocpp
