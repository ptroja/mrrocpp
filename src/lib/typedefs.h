// -------------------------------------------------------------------------
//                            typedefs.h
// Definicje typow calkowitych
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__TYPEDEFS_H) // Czy 'typedefs.h' juz wczytany ?
#define __TYPEDEFS_H

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#else
#include <stdint.h>
#include <signal.h>

#define _PULSE_CODE_MINAVAIL	0	/* QNX managers will never use this range */

struct _pulse {
	uint16_t	type;
	uint16_t	subtype;
	int8_t		code;
	uint8_t		zero[3];
	union sigval	value;
	int32_t		scoid;
};

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
#endif

#endif /* ! __QNXNTO__ */

typedef struct _pulse msg_header_t;

typedef struct  { // wiadomosc odbierana przez readera
    msg_header_t hdr;
    int data;	// TODO: to pole chyba nie jest potrzebne (ptroja)
} _pulse_msg;

#endif /* __TYPEDEFS_H */
