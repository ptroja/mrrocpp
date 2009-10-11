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

/* --- Symbolic names of the error return conditions --- */

#define EOK              0  /* No error                                 */

/*
 * Definitions for type and subtype of any pulse.
 * We keep the codes away from the defined SI_ signal codes to prevent
 * confusion.
 */
#define _PULSE_TYPE				0
#define _PULSE_SUBTYPE			0
#define _PULSE_CODE_UNBLOCK		(-32)	/* value - rcvid */
#define _PULSE_CODE_DISCONNECT	(-33)	/* value - server connection */
#define _PULSE_CODE_THREADDEATH	(-34)	/* value - thread id */
#define _PULSE_CODE_COIDDEATH	(-35)	/* value - thread id */
#define _PULSE_CODE_NET_ACK		(-36)	/* value - vtid; not in use anymore */
#define _PULSE_CODE_NET_UNBLOCK	(-37)	/* value - vtid */
#define _PULSE_CODE_NET_DETACH	(-38)	/* value - coid */

#define _PULSE_CODE_MINAVAIL	0	/* QNX managers will never use this range */
#define _PULSE_CODE_MAXAVAIL	127

/*
 *  Message types
 */
enum _msg_bases {
    _IO_BASE = 0x100,
    _IO_MAX = 0x1FF
};

struct _pulse {
	uint16_t	type;
	uint16_t	subtype;
	int8_t		code;
	uint8_t		zero[3];
	union sigval	value;
	int32_t		scoid;
};

struct _dispatch;
typedef struct _dispatch dispatch_t;

/*
 * Name dispatch functions
 */
#define NAME_FLAG_ATTACH_GLOBAL		0x00000002		/* Attach a global name */

typedef struct _name_attach {
    dispatch_t *dpp;
    int         chid;
    int         mntid;
	int			zero[2];
} name_attach_t;

name_attach_t *name_attach(dispatch_t *dpp, const char *path, unsigned flags);
int name_detach(name_attach_t *attach, unsigned flags);

int name_open(const char *name, int flags);
int name_close(int fd);

struct _msg_info {							/* _msg_info	_server_info */
	uint32_t					nd;			/*  client      server */
	uint32_t					srcnd;		/*  server      n/a */
	pid_t						pid;		/*	client		server */
	int32_t						tid;		/*	thread		n/a */
	int32_t						chid;		/*	server		server */
	int32_t						scoid;		/*	server		server */
	int32_t						coid;		/*	client		client */
	int32_t						msglen;		/*	msg			n/a */
	int32_t						srcmsglen;	/*	thread		n/a */
	int32_t						dstmsglen;	/*	thread		n/a */
	int16_t						priority;	/*	thread		n/a */
	int16_t						flags;		/*	n/a			client */
	uint32_t					reserved;
};

enum {
	SIGEV_UNBLOCK
};

extern int MsgSend(int __coid, const void *__smsg, int __sbytes, void *__rmsg, int __rbytes);
extern int MsgSendPulse(int __coid, int __priority, int __code, int __value);
extern int MsgReceive(int __chid, void *__msg, int __bytes, struct _msg_info *__info);
extern int MsgReply(int __rcvid, int __status, const void *__msg, int __bytes);

extern int ConnectDetach(int __coid);

extern int netmgr_strtond(const char *__nodename, char **__endstr);
extern int SignalKill(uint32_t __nd, pid_t __pid, int __tid, int __signo, int __code, int __value);

extern int TimerTimeout(int __id, int __flags, const struct sigevent *__notify,
		const uint64_t *__ntime, uint64_t *__otime);

extern uint64_t ClockCycles(void);

#define _NTO_TIMEOUT_RECEIVE	1
#define ND_LOCAL_NODE			0

#define delay(ms)	::usleep(1000*(ms))

#define flushall()	(void)0

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
