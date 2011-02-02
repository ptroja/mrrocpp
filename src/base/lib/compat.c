/*
 * OS compatibility layer
 */

#if (__APPLE__ & __MACH__)

#include <sys/time.h>
#include <errno.h>

// uncomment to show where the call originates from
//#define _ENABLE_DEBUG_BACKTRACE

#if defined(_ENABLE_DEBUG_BACKTRACE)
#include <execinfo.h>
#include <stdlib.h>
#include <stdio.h>
#endif /* _ENABLE_DEBUG_BACKTRACE */

#include "typedefs.h"

int clock_nanosleep(clockid_t clock_id, int flags,
       const struct timespec *rqtp, struct timespec *rmtp)
{
#if defined(_ENABLE_DEBUG_BACKTRACE)
	void* callstack[128];
	int i, frames = backtrace(callstack, 128);
	char** strs = backtrace_symbols(callstack, frames);
	for (i = 0; i < frames; ++i) {
		printf("%s\n", strs[i]);
	}
	free(strs);
#endif /* _ENABLE_DEBUG_BACKTRACE */
											 
	return EINVAL;
}

int clock_gettime( clockid_t clock_id, struct timespec * tp )
{
	struct timeval tv;

	/* Only CLOCK_REALTIME is emulated */
	if(clock_id != CLOCK_REALTIME) {
		errno = EINVAL;
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
