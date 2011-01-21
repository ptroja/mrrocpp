/*
 * OS compatibility layer
 */

#if (__APPLE__ & __MACH__)

#include <sys/time.h>

#include "typedefs.h"

int clock_nanosleep(clockid_t clock_id, int flags,
       const struct timespec *rqtp, struct timespec *rmtp)
{
	return -1;
}

int clock_gettime( clockid_t clock_id, struct timespec * tp )
{
	struct timeval tv;

	/* Only CLOCK_REALTIME is emulated */
	if(clock_id != CLOCK_REALTIME) {
		return -1;
	}

	if(gettimeofday(&tv, NULL) == -1) {
		return -1;
	}

	tp->tv_sec = tv.tv_sec;
	tp->tv_nsec = tv.tv_usec * 1000;

	return 0;
}

#endif /* (__APPLE__ & __MACH__) */
