/*!
 * @file typedefs.h
 * @brief Base typedefs for OS compatibility and communication structures.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#if !defined(__TYPEDEFS_H)
#define __TYPEDEFS_H

/* --- Symbolic names of the error return conditions --- */

#define EOK              0  /* No error */

#define _PULSE_CODE_MINAVAIL	0	/* QNX managers will never use this range */

#define ND_LOCAL_NODE			0

#define delay(ms)	usleep(1000*(ms))

#define flushall()	do { fflush(stdout); fflush(stderr); } while (0)

#if defined(linux)
#define	out8(port,val)	outb((val),(port))
#define	out16(port,val)	outw((val),(port))
#define	in8(port)		inb(port)
#define	in16(port)		inw(port)
#elif defined(__FreeBSD__)
#include <machine/sysarch.h>
//#include <machine/cpufunc.h>
#define	out8(port,val)	outb((port),(val))
#define	out16(port,val)	outw((port),(val))
#define	in8(port)		inb(port)
#define	in16(port)		inw(port)
#elif defined(sun)
/*
 // These functions are obsolete, but more handy; use ddi_put8(9F) and similar instead
 #include <sys/ddi.h>
 #include <sys/sunddi.h>
 #define	out8(port,val)	outb((port),(val))
 #define	out16(port,val)	outw((port),(val))
 #define	in8(port)		inb(port)
 #define	in16(port)		inw(port)
 */
#define	out8(port,val)	(void) 0
#define	out16(port,val)	(void) 0
#define	in8(port)		0
#define	in16(port)		0
#elif (__APPLE__ & __MACH__)

/* MacOS X does not provide a clock_gettime(), but it is easy to replace */
typedef int clockid_t;
/* Identifier for system-wide realtime clock.  */
#   define CLOCK_REALTIME       0
/* Monotonic system-wide clock.  */
#   define CLOCK_MONOTONIC      1
/* High-resolution timer from the CPU.  */
#   define CLOCK_PROCESS_CPUTIME_ID 2
/* Thread-specific CPU-time clock.  */
#   define CLOCK_THREAD_CPUTIME_ID  3

/* Flag to indicate time is absolute.  */
#   define TIMER_ABSTIME        1

#ifdef __cplusplus
extern "C" {
#endif

	int clock_gettime( clockid_t clock_id, struct timespec * tp );

	/* declaration only; not replaced yet, but this is used only for robot hardware drivers */
	int clock_nanosleep(clockid_t clock_id, int flags,
			const struct timespec *rqtp, struct timespec *rmtp);

#ifdef __cplusplus
}
#endif

#define	out8(port,val)	(void) 0
#define	out16(port,val)	(void) 0
#define	in8(port)		0
#define	in16(port)		0

#endif

#endif /* __TYPEDEFS_H */
