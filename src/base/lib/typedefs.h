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

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>

typedef struct _pulse msg_header_t;

//! Pulse data structure
typedef struct
{
	msg_header_t hdr;
	int data; // TODO: this probably is not needed anymore
} _pulse_msg;

#else
#include <stdint.h>
#include <csignal>

/* --- Symbolic names of the error return conditions --- */

#define EOK              0  /* No error */

#define _PULSE_CODE_MINAVAIL	0	/* QNX managers will never use this range */

#define ND_LOCAL_NODE			0

#define delay(ms)	::usleep(1000*(ms))

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
#endif

#endif /* ! __QNXNTO__ */

#endif /* __TYPEDEFS_H */
