/*
 MessIP : Message Passing over TCP/IP
 Copyright (C) 2001,2002,2003,2004  Olivier Singla
 olivier@singla.us
 http://singla.us/messip/

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */

#ifndef __MESSIP__
#define __MESSIP__

#include <pthread.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <signal.h>
#include <time.h>

//#define USE_MESSIP_SRR

#if defined(__linux__)
//#define USE_SRRMOD	1
#endif /* __linux__ */



#ifdef USE_QNXMSG
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif /* USE_QNXMSG */

#define CHANNEL_TYPE_MESSIP		0
#define CHANNEL_TYPE_SRRMOD		1
#define CHANNEL_TYPE_QNXMSG		2

#define MESSIP_ETC	"/etc/messip"
#define	MESSIP_DEFAULT_PORT	9200

#define MESSIP_SOCK_DOMAIN	AF_INET
#define MESSIP_SOCK_TYPE	SOCK_STREAM

// Uncomment the line below for use of Stream Control Transmission Protocol
// instead of plain old TCP
//#define MESSIP_USE_SCTP

#ifndef MESSIP_USE_SCTP
#define MESSIP_SOCK_PROTO	0
#define MESSIP_NODELAY_LEVEL	IPPROTO_TCP
#define MESSIP_NODELAY_OPTNAME	TCP_NODELAY
#else
#if defined(__linux__)
#include <netinet/sctp.h>
#endif
#define MESSIP_SOCK_PROTO	IPPROTO_SCTP
#define MESSIP_NODELAY_LEVEL	SOL_SCTP
#define MESSIP_NODELAY_OPTNAME	SCTP_NODELAY
#endif

#define VERSION_MAJOR	0
#define	VERSION_MINOR	8
#define VERSION_PATCH	5

#define MESSIP_OK	1
#define MESSIP_NOK	0

#define MESSIP_TRUE		-1
#define MESSIP_FALSE	0

#define	MESSIP_MAXLEN_TASKNAME		15
#define	MESSIP_CHANNEL_NAME_MAXLEN	47
#define	MESSIP_QNXNODE_NAME_MAXLEN	47

/*
 * This is a linked list, as we can have connection to multiple servers
 */
typedef struct messip_cnx
{
	//	char path[80];								// "/' if only server
	int sockfd;
	pid_t remote_pid;
	pthread_t remote_tid;
	struct messip_cnx *prev;
} messip_cnx_t;

typedef struct messip_channel
{
	char name[MESSIP_CHANNEL_NAME_MAXLEN + 1];
	messip_cnx_t *cnx;
	int32_t f_already_connected;
	pid_t remote_pid;
	char remote_taskname[MESSIP_CHANNEL_NAME_MAXLEN + 1];
	pthread_t remote_tid;
	int32_t recv_sockfd_sz;
	int recv_sockfd[FD_SETSIZE];
	int remote_port;
	in_port_t sin_port;
	in_addr_t sin_addr;
	//char sin_addr_str[48];
	char hostname[48];
	int send_sockfd;
	int *new_sockfd;
	int *channel_type;
	int32_t new_sockfd_sz;
	int32_t nb_replies_pending; // Nb of receive() without reply()
	int32_t datalen; // Length of data transmitted
	int32_t datalenr; // Length of data read
	void **receive_allmsg; // Dynamic buffer, if receive buffer was to small
	int *receive_allmsg_sz; // Size allocated for these Dynamic buffer
	int nb_timers;
	int mgr_sockfd; // Socket in the messip_mgr
	int lastmsg_sockfd; // socket number of the last message, for QNX scoid comptibility
	pthread_mutex_t send_mutex; // mutex for exclusion of simultanous write by different threads
#ifdef USE_SRRMOD
	int srr_name_id;
	int srr_pid;
#endif /* USE_SSRMOD */
#ifdef USE_QNXMSG
	resmgr_connect_funcs_t ConnectFuncs;
	resmgr_io_funcs_t IoFuncs;
	iofunc_attr_t IoFuncAttr;
	resmgr_attr_t resmgr_attr;
	message_attr_t message_attr;
	int resmgr_id;
	int fd; // open decriptor
	dispatch_t *dpp; // select/message dispatch
	dispatch_context_t *ctp; // select/message dispatch context
	dispatch_context_t *new_ctp; // select/message dispatch context
	int msgcnt, selcnt;
	int index;
#endif /* USE_QNXMSG */
} messip_channel_t;

typedef struct messip_message_handler
{
	messip_channel_t *ch;
	int (*func)(messip_channel_t * ch, void * arg);
	void * arg;
} messip_message_handler_t;

typedef struct messip_dispatch
{
	struct timeval timeout;
	fd_set ready;
	messip_message_handler_t *handlers;
	unsigned int nb_handlers;
} messip_dispatch_t;

#define MESSIP_MSG_DISCONNECT		-2
#define MESSIP_MSG_DISMISSED		-3
#define MESSIP_MSG_TIMEOUT			-4
#define MESSIP_MSG_TIMER			-5
#define MESSIP_MSG_NOREPLY			-6
#define MESSIP_MSG_DEATH_PROCESS	-7
#define MESSIP_MSG_CONNECTING		-8

// -----------------------
// Prototypes of functions
// -----------------------

#define MESSIP_NOTIMEOUT		-1

#ifdef __cplusplus
extern "C" {
#endif

float get_cpu_clock_speed(void);

void messip_init(void);
void qmpw_init();
messip_cnx_t *messip_connect(const char *mgr_ref, int msec_timeout);
messip_cnx_t *messip_connect0(const char *mgr_ref, int msec_timeout, pid_t pid, pthread_t tid);
int
messip_sin(char *mgr_ref);
messip_channel_t
		*messip_channel_create(messip_cnx_t * cnx, const char *name, int msec_timeout, int32_t maxnb_msg_buffered);
int messip_channel_delete(messip_channel_t * ch, int msec_timeout);
messip_channel_t *messip_channel_connect(messip_cnx_t * cnx, const char *name, int msec_timeout);
int messip_channel_disconnect(messip_channel_t * ch, int msec_timeout);
int messip_channel_ping(messip_channel_t * ch, int msec_timeout);
int
		messip_receive(messip_channel_t * ch, int32_t * type, int32_t * subtype, void *buffer, int32_t maxlen, int msec_timeout);
int
		messip_reply(messip_channel_t * ch, int index, int32_t answer, const void *reply_buffer, int reply_len, int msec_timeout);
int
		messip_send(messip_channel_t * ch, int32_t type, int32_t subtype, const void *send_buffer, int send_len, int32_t * answer, void *reply_buffer, int reply_maxlen, int msec_timeout);
int32_t
		messip_buffered_send(messip_channel_t * ch, int32_t type, int32_t subtype, void *send_buffer, int send_len, int msec_timeout);
#if !defined(__FreeBSD__) && !(__APPLE__ & __MACH__)
timer_t
	messip_timer_create(messip_channel_t * ch, int32_t type, int32_t subtype, int32_t msec_1st_shot, int32_t msec_rep_shot, int msec_timeout);
		
int messip_timer_delete(messip_channel_t * ch, timer_t timer_id);
#endif
int messip_death_notify(messip_cnx_t * cnx, int msec_timeout, int status);

typedef int (*messip_callback_t)(messip_channel_t * ch, void * arg);

messip_dispatch_t *messip_dispatch_create(void);
void messip_dispatch_delete(messip_dispatch_t *dpp);
int messip_dispatch_attach(messip_dispatch_t *dpp, messip_channel_t * ch,
//		int (*func) (messip_channel_t * ch, void * arg),
messip_callback_t func, void * arg);
//int messip_dispatch_dettach(messip_dispatch_t *dispatch, messip_channel_t * ch);
int messip_dispatch_block(messip_dispatch_t *dpp, int msec_timeout);
int messip_dispatch_handler(messip_dispatch_t *dpp);

#ifdef __cplusplus
}
#endif

#endif
