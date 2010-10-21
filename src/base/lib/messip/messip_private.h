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


#if defined(PRINT_TRACE)
#	define TRACE(fmt, args...)	printf( fmt, ## args )
#else
#	define TRACE(fmt, args...)
#endif


#if 0
/*
	FreeBSD is missing these definitions
*/

#if defined(__FreeBSD__)

#ifndef SIGEV_THREAD
#	define SIGEV_THREAD 2
#endif

typedef union sigval sigval_t;

struct itimerspec
{
	struct timespec it_interval;	// timer period
	struct timespec it_value;		// timer expiration
};

#endif
#endif


/*
*/

#if defined(__linux__) || defined(__QNX__) || defined(sun)
#	define SIGVAL_PTR sival_ptr
#elif defined(__FreeBSD__) || (defined(__APPLE__) && defined(__MACH__))
#	define SIGVAL_PTR sigval_ptr
#else
#error Unsupported platform!
#endif


//
// /etc/messip
// 

typedef struct messip_mgr
{
	char host[80];
	int port;
	char path[80];
	struct messip_mgr *next;
}
messip_mgr_t;


// op : int32_t
enum
{
	MESSIP_OP_CONNECT = 1,
	MESSIP_OP_CHANNEL_CREATE,
	MESSIP_OP_CHANNEL_DELETE,
	MESSIP_OP_CHANNEL_CONNECT,
	MESSIP_OP_CHANNEL_DISCONNECT,
	MESSIP_OP_CHANNEL_PING,
	MESSIP_OP_BUFFERED_SEND,
	MESSIP_OP_DEATH_NOTIFY,
	
	MESSIP_OP_SIN = 100,

#ifdef MESSIP_INFORM_STATE
	MESSIP_DEBUG_OP_INFORM_STATE = 200,
#endif

	MESSIP_QNX_OP_PROXY_ATTACH = 1000,
	MESSIP_QNX_OP_PROXY_DETACH,
	MESSIP_QNX_OP_PROXY_TRIGGER,
	MESSIP_QNX_OP_PROXY_GET_OWNER

};


// -------------------------
// MESSIP_OP_CONNECT message
// -------------------------

typedef struct
{
	int32_t little_endian;
	pid_t pid;
	pthread_t tid;
	char process_name[MESSIP_MAXLEN_TASKNAME+1];	// Process name (read from /proc/pid/stat
}
messip_send_connect_t;

typedef struct
{
	int32_t ok;
}
messip_reply_connect_t;


// ---------------------------------------
// MESSIP_OP_CHANNEL_CREATE channel_create
// ---------------------------------------

typedef struct
{
	pid_t pid;
	pthread_t tid;
	int32_t maxnb_msg_buffered;
	char channel_name[MESSIP_CHANNEL_NAME_MAXLEN + 1];
	char qnxnode_name[MESSIP_QNXNODE_NAME_MAXLEN + 1];
	uint16_t sin_port;
	char sin_addr_str[48];
} messip_send_channel_create_t;

typedef struct
{
	int32_t ok;
	in_port_t sin_port;
	in_addr_t sin_addr;
	char sin_addr_str[48];
} messip_reply_channel_create_t;


// ----------------------------------------
// MESSIP_OP_CHANNEL_DELETE channel_delete
// ----------------------------------------

typedef struct
{
	pid_t pid;
	pthread_t tid;
	char name[MESSIP_CHANNEL_NAME_MAXLEN + 1];
} messip_send_channel_delete_t;

typedef struct
{
	int32_t nb_clients;
} messip_reply_channel_delete_t;

// ----------------------------------------
// MESSIP_OP_CHANNEL_CONNECT channel_connect
// ----------------------------------------

typedef struct
{
	pid_t pid;
	pthread_t tid;
	char name[MESSIP_CHANNEL_NAME_MAXLEN + 1];
} messip_send_channel_connect_t;

typedef struct
{
	int32_t ok;
	int32_t f_already_connected;
	pid_t pid;
	pthread_t tid;
	in_port_t sin_port;						// 2 bytes
	in_addr_t sin_addr;						// 4 bytes
	char sin_addr_str[48];
	int mgr_sockfd;							// Socket in the messip_mgr
	char qnxnode_name[MESSIP_QNXNODE_NAME_MAXLEN + 1];
} messip_reply_channel_connect_t;


// -----------------------------------------------
// MESSIP_OP_CHANNEL_DISCONNECT channel_disconnect
// -----------------------------------------------

typedef struct
{
	pid_t pid;
	pthread_t tid;
	char name[MESSIP_CHANNEL_NAME_MAXLEN + 1];
}
messip_send_channel_disconnect_t;

typedef struct
{
	int32_t ok;
}
messip_reply_channel_disconnect_t;


// -----------------------------------
// MESSIP_OP_CHANNEL_PING channel_ping
// -----------------------------------

typedef struct
{
	pid_t pid;
	pthread_t tid;
}
messip_send_channel_ping_t;

typedef struct
{
	int32_t ok;								   // MESSIP_OK or MESSIP_NOK
}
messip_reply_channel_ping_t;


// -------------------------------------
// MESSIP_OP_BUFFERED_SEND buffered_send
// -------------------------------------

typedef struct
{
	pid_t pid_from;
	pthread_t tid_from;
	int32_t type,
	  subtype;
	int32_t datalen;
	int mgr_sockfd;								// Socket in the messip_mgr
}
messip_send_buffered_send_t;

typedef struct
{
	int32_t ok;									// MESSIP_OK or MESSIP_NOK
	int32_t nb_msg_buffered;
}
messip_reply_buffered_send_t;


// -----------------------------------
// MESSIP_QNX_OP_PROXY_ATTACH 
// -----------------------------------

typedef struct
{
	pid_t pid_from;
	pthread_t tid_from;
	pid_t pid_to;
	char *data;
	int nbytes;
	int priority;
}
messip_send_proxy_attach_t;

typedef struct
{
	int32_t ok;									// MESSIP_OK or MESSIP_NOK
	pid_t proxy;								// Index into proxies[] table
}
messip_reply_proxy_attach_t;


// -----------------------------------
// MESSIP_QNX_OP_PROXY_DETACH 
// -----------------------------------

typedef struct
{
	pid_t pid_from;
	pthread_t tid_from;
	pid_t proxy;								// Index into proxies[] table
}
messip_send_proxy_detach_t;

typedef struct
{
	int32_t ok;									// MESSIP_OK or MESSIP_NOK
}
messip_reply_proxy_detach_t;


/// -----------------------------------
// MESSIP_QNX_OP_PROXY_TRIGGER 
// -----------------------------------

typedef struct
{
	pid_t pid_from;
	pthread_t tid_from;
	pid_t pid_to;
}
messip_send_proxy_trigger_t;

typedef struct
{
	int32_t ok;								   // MESSIP_OK or MESSIP_NOK
	pid_t pid_owner;						   // PID of process owning the proxy
}
messip_reply_proxy_trigger_t;


/// -----------------------------------
// MESSIP_QNX_OP_PROXY_GET_OWNER 
// -----------------------------------

typedef struct
{
	pid_t pid_from;
	pthread_t tid_from;
	pid_t pid_to;
}
messip_send_proxy_get_owner_t;

typedef struct
{
	int32_t ok;								   // MESSIP_OK or MESSIP_NOK
	pid_t pid_owner;						   // PID of process owning the proxy
}
messip_reply_proxy_get_owner_t;


// ----------------------
// MESSIP_OP_DEATH_NOTIFY 
// ----------------------

typedef struct
{
	pid_t pid_from;
	pthread_t tid_from;
	int32_t status;
}
messip_send_death_notify_t;

typedef struct
{
	int32_t ok;								   // MESSIP_OK or MESSIP_NOK
}
messip_reply_death_notify_t;


// ----------------------------------------------
// Additional information sent on a messip_send()
// ----------------------------------------------

#define MESSIP_FLAG_CONNECTING		1
#define MESSIP_FLAG_DISCONNECTING	2
#define MESSIP_FLAG_DISMISSED		3
#define MESSIP_FLAG_TIMER			5
#define MESSIP_FLAG_BUFFERED		6
#define MESSIP_FLAG_PING			7
#define MESSIP_FLAG_PROXY			8
#define MESSIP_FLAG_DEATH_PROCESS	9
#define MESSIP_FLAG_1WAY_MESSAGE	10

typedef struct
{
	int32_t flag;
	pid_t pid;
	pthread_t tid;
	int32_t type,
	  subtype;
	int32_t datalen;
}
messip_datasend_t;

typedef struct
{
	pid_t pid;
	pthread_t tid;
	int32_t answer;
	int32_t datalen;
}
messip_datareply_t;


#ifdef MESSIP_INFORM_STATE

// ---------------------------------------------
// State of the Processes (In DEBUG mode *only*)
// ---------------------------------------------

#define MESSIP_STATE_NIL				1
#define MESSIP_STATE_SEND_BLOCKED		2
#define MESSIP_STATE_REPLY_BLOCKED		3
#define MESSIP_STATE_RECEIVE_BLOCKED	4

typedef struct
{
	pid_t pid;
	pthread_t tid;
	int32_t status;
	pid_t pid_blocked_on;
	pthread_t tid_blocked_on;
	time_t when_blocked_on;
}
messip_send_inform_messipmgr_t;

typedef struct
{
	int32_t ok;
}
messip_reply_inform_messipmgr_t;

#endif


// --------------------------
// Functions used by qmpw_lib
// --------------------------

int messip_writev( int sockfd,
   const struct iovec *iov,
   int iovcnt );
int messip_readv( int sockfd,
   const struct iovec *iov,
   int iovcnt );
int messip_int_little_endian( const int v1 );

enum
{
	STATE1 = 101,
	STATE2 = 202,
};
