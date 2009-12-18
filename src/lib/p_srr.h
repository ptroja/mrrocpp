#ifndef _P_SRR_H
#define _P_SRR_H	1

#include <pthread.h>
#include <semaphore.h>

#define USE_QNXMSG	1
//#define USE_SRRMOD	0
//#define USE_MESSIP	0

#ifdef USE_SRRMOD
#include <srr.h>
#define P_SRR_SRRMOD_SUPPORTED	1
#else
#define P_SRR_SRRMOD_SUPPORTED	0
#endif /* USE_SRRMOD */

#ifdef USE_MESSIP
#include <messip.h>
#define P_SRR_MESSIP_SUPPORTED	1
#else
#define P_SRR_MESSIP_SUPPORTED	0
#endif /* USE_MESSIP */

#ifdef USE_QNXMSG
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#define P_SRR_QNXMSG_SUPPORTED	1
#else
#define P_SRR_QNXMSG_SUPPORTED	0
#endif /* USE_QNXMSG */

#define P_SRR_PROTOCOLS_SUPPORTED	(P_SRR_SRRMOD_SUPPORTED+P_SRR_MESSIP_SUPPORTED+P_SRR_QNXMSG_SUPPORTED)

#if (P_SRR_PROTOCOLS_SUPPORTED == 0)
#error "P_SRR_PROTOCOLS_SUPPORTED == 0"
#endif

#define P_SRR_MESSIP_PROTOCOL	1
#define P_SRR_SRRMOD_PROTOCOL	2
#define P_SRR_QNXMSG_PROTOCOL	3

typedef struct {
    struct {
	sem_t defined;
	char attached;
    } msgsize;
#ifdef USE_SRRMOD
    struct {
	pthread_t srr_thread;
	int name_id;
	pid_t me;
	void * msg;
	pid_t from;
    } srr;
#endif /* USE_SRRMOD */
#ifdef USE_MESSIP
    struct {
	pthread_t messip_thread;
	messip_channel_t *channel;
	void * msg;
	int index;
    } messip;
#endif /* USE_MESSIP */
#ifdef USE_QNXMSG
	struct {
		pthread_t qnxmsg_thread;
		name_attach_t *na;
		void * msg;
	} qnxmsg;
#endif /* USE_QNXMSG */
    char *path;
    int bytes;
    void * msg;
    int rbytes;
    void * rmsg;
    int reply_ret;
    pthread_mutex_t process_msg;
    sem_t received, reply_ready, replied, attach_protocol;
} p_name_attach_t;

typedef struct {
    char protocol;
    union {
#ifdef USE_SRRMOD
	pid_t pid;
#endif /* USE_SRRMOD */
#ifdef USE_MESSIP
	messip_channel_t *messip_channel;
#endif /* USE_MESSIP */
#ifdef USE_QNXMSG
	int fd;
#endif /* USE_QNXMSG */
    } node_id;
} p_channel_t;

extern
p_name_attach_t *
p_name_attach(const char *path);

extern
int
p_name_detach(p_name_attach_t * attach);

extern
p_channel_t *
p_name_open(const char *path);

extern
int
p_name_close(p_channel_t *channel);

extern
int
p_MsgSend(p_channel_t *channel, const void *smsg, int sbytes, void *rmsg, int rbytes);

extern
int
p_MsgReceive(p_name_attach_t *attach, void * msg, int bytes);

extern
int
p_MsgReply(p_name_attach_t *attach, void * msg, int bytes);

#endif	/* p_srr.h */
