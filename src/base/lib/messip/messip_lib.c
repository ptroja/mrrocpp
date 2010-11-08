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


/*
	messip_lib		Olivier Singla - olivier@singla.us
					2001, 2002, 2003

	This is the library that encapsulate the communication with the
	manager (messip_mgr). The library also implements send/receive/reply
	because the manager is *not* then involved  (unless it's a buffered
	message or a qnx4-like proxy).
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <fcntl.h>
#include <signal.h>
#include <assert.h>
#include <sys/select.h>
#include <netinet/tcp.h>
#if defined(__linux__)
#include <endian.h>
#elif defined(__FreeBSD__)
#include <sys/endian.h>
#elif defined(__APPLE__)  && defined(__MACH__)
#include <machine/endian.h>
#elif defined(__QNX__)
#include <sys/param.h>
#include <sys/netmgr.h>
#elif defined(sun)
#else
#error Unsupported platform!
#endif

#ifdef __gnu_linux__
#include <execinfo.h>
#endif /* __gnu_linux__ */

#include "messip.h"
#include "messip_private.h"
#include "messip_utils.h"

#ifdef USE_SRRMOD
#include <srr.h>
static int use_srrmod;
#endif /* USE_SRRMOD */

#ifdef USE_QNXMSG
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif /* USE_QNXMSG */

#define LIBTRACE( X )
//#define LIBTRACE( X ) printf X

typedef struct
{
	char name[MESSIP_CHANNEL_NAME_MAXLEN + 1];
	messip_channel_t *info;
}
list_connect_t;
static list_connect_t *list_connect = NULL;
static int nb_list_connect = 0;

/*
	This function will be executed automatically before main() start
		(ELF format feature)
*/

void
messip_init( void )
{
	list_connect = NULL;
	nb_list_connect = 0;
#ifdef USE_SRRMOD
	if (SrrReg() == -1) {
	    perror("SrrReg()");
	    use_srrmod = 0;
	} else {
	    use_srrmod = 1;
	}
#endif /* USE_SRRMOD */
}								// messip_init

static pthread_once_t messip_cnx_is_initialized = PTHREAD_ONCE_INIT;
static pthread_mutex_t messip_cnx_mutex = PTHREAD_MUTEX_INITIALIZER;

static messip_cnx_t *messip_cnx_private;

void
messip_cnx_init(void)
{
	messip_init();
	//messip_cnx_private = messip_connect(NULL, MESSIP_NOTIMEOUT);
	messip_cnx_private = messip_connect(getenv("UI_HOST"), MESSIP_NOTIMEOUT);
	return;
}

#if 0
#if defined( __STATIC__ )

/*
	Static Library
	This function will be executed automatically before main() start
		(ELF format feature)
*/

void do_init(  ) __attribute__ ( ( constructor ) );
void
do_init(  )
{
	messip_init();
}								// do_init

#elif defined( __DYNAMIC__ )

/*
	Dynamic Library
	This function will be executed automatically before main() start
		(Dynamic Loader feature)
*/

void
_init(  )
{
	messip_init();
}

#else

#error Neither __STATIC__ or __DYNAMIC__

#endif
#endif


int
messip_writev( int sockfd,
   const struct iovec *iov,
   int iovcnt )
{
	int dcount;
	int cnt = 0;

	for ( ;; )
	{
		dcount = writev( sockfd, iov, iovcnt );
		if ( (dcount == -1) && (errno == EINTR) )
			continue;
		if ( dcount == -1 )
			fprintf( stderr, "%d: %s %d: dcount=%d errno=%d (%s) fileno: %d %d\n",
			   getpid(), __FILE__, __LINE__, dcount, errno, strerror(errno), sockfd, cnt++ );
		if ( errno == EPIPE )
			return dcount;
		//assert( dcount != -1 );
		if (!(dcount != -1)) {
			fprintf(stderr, "assert( dcount != -1 ) failed at %s:%d\n", __FILE__, __LINE__);
		}
		break;
	}							// for (;;)

	return dcount;

}								// messip_writev


int
messip_readv( int sockfd,
   const struct iovec *iov,
   int iovcnt )
{
	int dcount;
	int cnt = 0;

	for ( ;; )
	{
		dcount = readv( sockfd, iov, iovcnt );
		if ( (dcount == -1) && (errno == EINTR) )
			continue;
		if ( (dcount == -1) && (errno == ECONNRESET) )
			return -1;
		if ( dcount == -1 )
			fprintf( stderr, "%d: %s %d: dcount=%d errno=%d %d\n",
			   getpid(), __FILE__, __LINE__, dcount, errno, cnt++ );
		assert( dcount != -1 );
		break;
	}							// for (;;)

	return dcount;

}								// messip_readv


int
messip_select( int fd,
   fd_set *readfds,
   fd_set *writefds,
   fd_set *exceptfds,
   struct timeval *timeout )
{
	int	status;

	do
	{
		status = select( fd, readfds, writefds, exceptfds, timeout );
		if ( (status < 0) && (status != EINTR) )
			return status;
	} while ( status < 0 );

	return status;

}								// messip_select


int
messip_int_little_endian( const int v1 )
{
#if BYTE_ORDER == BIG_ENDIAN
	unsigned char *p,
	 *q;
	int v2;

	p = ( unsigned char * ) &v1;
	q = ( unsigned char * ) &v2;
	q[3] = p[0];
	q[2] = p[1];
	q[1] = p[2];
	q[0] = p[3];
	return v2;
#elif BYTE_ORDER == LITTLE_ENDIAN
	return v1;
#else
#error
#endif
}								// messip_int_little_endian


messip_cnx_t *
messip_connect0(const char *mgr_ref,
   int32_t msec_timeout,
   pid_t pid,
   pthread_t tid )
{
	messip_cnx_t *cnx;
	struct sockaddr_in server;
	struct hostent *hp;
	char hostname[64];
	int port;
	int status;
	fd_set ready;
	struct timeval tv;
	int fcntlArg;
	int32_t op;
	messip_send_connect_t msgsend;
	messip_reply_connect_t reply;
	struct iovec iovec[2];
	int flag = 0;

#if BYTE_ORDER == BIG_ENDIAN
	pid = messip_int_little_endian( pid );
	tid = messip_int_little_endian( tid );
#endif

	/*--- NULL and /etc/messip does not exist ? ---*/
	port = MESSIP_DEFAULT_PORT;
	if ( !mgr_ref && ( access( MESSIP_ETC, F_OK ) == -1 ) )
	{
		strcpy( hostname, "localhost" );
	}
	else if ( !mgr_ref )
	{
		read_etc_messip( hostname, &port, NULL );
	}
	else
	{
		strcpy( hostname, mgr_ref );
	}

	/*--- Connect socket using name specified ---*/
	server.sin_family = AF_INET;
	hp = gethostbyname( hostname );
	if ( hp == NULL )
	{
		fprintf( stderr, "*** %s : unknown host!***\n", hostname );
		return NULL;
	}

	/*--- Allocate a connexion structure ---*/
	cnx = (messip_cnx_t *) malloc( sizeof( messip_cnx_t ) );
	memset( cnx, 0, sizeof( messip_cnx_t ) );

	/*--- Create socket ---*/
	cnx->sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if ( cnx->sockfd < 0 )
	{
		perror("socket()");
		free( cnx );
		return NULL;
	}
	fcntl( cnx->sockfd, F_SETFL, FD_CLOEXEC );
//	logg( NULL, "%s: sockfd=%d\n", __FUNCTION__, cnx->sockfd );

	/* Disable the Nagle (TCP No Delay) algorithm */
	if (setsockopt( cnx->sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int) ) == -1) {
	  perror("setsockopt(TCP_NODELAY)");
	}

	/*
	 * Change to non-blocking mode to enable timeout with connect()
	 */
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		if ( ( ( fcntlArg = fcntl( cnx->sockfd, F_GETFL ) ) == -1 ) ||
		   ( fcntl( cnx->sockfd, F_SETFL, fcntlArg & ~O_NDELAY ) == -1 ) )
		{
			fprintf( stderr, "Error: fcntl() !\n" );
			close( cnx->sockfd );
			free( cnx );
			return NULL;
		}
	}							// if

	/*--- Connect to the port ---*/
	memcpy( &server.sin_addr, hp->h_addr, hp->h_length );
	server.sin_port = htons( port );
	status = connect( cnx->sockfd, ( const struct sockaddr * ) &server, sizeof( server ) );
	if ( ( msec_timeout != MESSIP_NOTIMEOUT ) && ( status ) )
	{
		if ( errno == EINPROGRESS )
		{
			FD_ZERO( &ready );
			FD_SET( cnx->sockfd, &ready );
			tv.tv_sec = msec_timeout / 1000;
			tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
			status = select( cnx->sockfd+1, NULL, &ready, NULL, &tv );	// <&>
			assert( status != -1 );
			if ( !FD_ISSET( cnx->sockfd, &ready ) )
			{
				close( cnx->sockfd );
				free( cnx );
				errno = ETIMEDOUT;
				return NULL;
			}
		}
		else
		{
			fprintf( stderr, "%s %d:\n\tUnable to connect to host %s (%s), port %d\n",
			   __FILE__, __LINE__, hostname, inet_ntoa(server.sin_addr), port );
			close( cnx->sockfd );
			free( cnx );
			return NULL;
		}
	}
	else if ( status )			// if
	{
		fprintf( stderr, "%s %d:\n\tUnable to connect to host %s (%s), port %d\n",
		   __FILE__, __LINE__, hostname, inet_ntoa(server.sin_addr), port );
		close( cnx->sockfd );
		free( cnx );
		return NULL;
	}

	/*--- Ready to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( cnx->sockfd+1, NULL, &ready, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			close( cnx->sockfd );
			free( cnx );
			errno = ETIMEDOUT;
			return NULL;
		}
	}							// if

	/*--- Send a message to the server ---*/
	iovec[0].iov_base = &op;
	iovec[0].iov_len  = sizeof( int32_t );
	memset(&msgsend, 0, sizeof(msgsend));
#if BYTE_ORDER == LITTLE_ENDIAN
	op = MESSIP_OP_CONNECT;
	msgsend.little_endian = 1;
#elif BYTE_ORDER == BIG_ENDIAN
	op = messip_int_little_endian( MESSIP_OP_CONNECT );
	msgsend.little_endian = 0;
#else
#	error
#endif
	msgsend.pid = pid;
	msgsend.tid = tid;
	get_taskname( pid, msgsend.process_name );
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	status = messip_writev( cnx->sockfd, iovec, 2 );
	assert( ( status == sizeof( int32_t ) + sizeof( msgsend ) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( cnx->sockfd, &ready, NULL, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			close( cnx->sockfd );
			free( cnx );
			errno = ETIMEDOUT;
			return NULL;
		}
	}							// if

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	status = messip_readv( cnx->sockfd, iovec, 1 );
	assert( status == sizeof( messip_reply_connect_t ) );
	assert( reply.ok == MESSIP_OK );
	cnx->remote_pid = pid;
	cnx->remote_tid = tid;

	/*--- Ok ---*/
	return cnx;

}								// messip_connect0


messip_cnx_t *
messip_connect(const char *mgr_ref,
   int32_t msec_timeout )
{
	return messip_connect0( mgr_ref, msec_timeout, getpid(  ), pthread_self(  ) );
}								// messip_connect


int
messip_sin( char *mgr_ref )
{
	struct sockaddr_in server;
	struct hostent *hp;
	char hostname[64];
	int port;
	int status;
	int32_t op;
	struct iovec iovec[1];
	int sockfd;
	int flag = 0;

	/*--- NULL and /etc/messip does not exist ? ---*/
	port = MESSIP_DEFAULT_PORT;
	if ( !mgr_ref && ( access( MESSIP_ETC, F_OK ) == -1 ) )
	{
		strcpy( hostname, "localhost" );
	}
	else if ( !mgr_ref )
	{
		read_etc_messip( hostname, &port, NULL );
	}
	else
	{
		strcpy( hostname, mgr_ref );
	}

	/*--- Connect socket using name specified ---*/
	server.sin_family = AF_INET;
	hp = gethostbyname( hostname );
	if ( hp == NULL )
	{
		fprintf( stderr, "*** %s : unknown host!***\n", hostname );
		return -1;
	}

	/*--- Create socket ---*/
	sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if ( sockfd < 0 )
	{
		perror("socket()");
		return -1;
	}
	fcntl( sockfd, F_SETFL, FD_CLOEXEC );

	/* Disable the Nagle (TCP No Delay) algorithm */
	if (setsockopt( sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int) ) == -1) {
	  perror("setsockopt(TCP_NODELAY)");
	}

	/*--- Connect to the port ---*/
	memcpy( &server.sin_addr, hp->h_addr, hp->h_length );
	server.sin_port = htons( port );
	status = connect( sockfd, ( const struct sockaddr * ) &server, sizeof( server ) );
	if ( status == -1 )
	{
		fprintf( stderr, "%s %d:\n\tUnable to connect to host %s (%s), port %d\n",
		   __FILE__, __LINE__, hostname, inet_ntoa(server.sin_addr), port );
		if ( close( sockfd ) == -1 )
			fprintf( stderr, "Error %d while closing socket %d\n",
				errno, sockfd );
		return -1;
	}

	/*--- Send a message to the server ---*/
#if BYTE_ORDER == LITTLE_ENDIAN
	op = MESSIP_OP_SIN;
#elif BYTE_ORDER == BIG_ENDIAN
	op = messip_int_little_endian( MESSIP_OP_SIN );
#else
#error
#endif
	iovec[0].iov_base = &op;
	iovec[0].iov_len  = sizeof( int32_t );
	status = messip_writev(sockfd, iovec, 1 );
	assert( ( status == sizeof( int32_t ) ) );
	if ( close( sockfd ) == -1 )
		fprintf( stderr, "Error %d while closing socket %d\n",
			errno, sockfd );

	/*--- Ok ---*/
	return 0;

}								// messip_sin

#ifdef USE_QNXMSG
static int
message_handler(message_context_t *ctp, int code,
                unsigned flags, void *handle ) {

	messip_channel_t *ch = (messip_channel_t *) handle;

	//printf("mh: %s\n", ((char *) ctp->msg) + sizeof(uint16_t));

	ch->msgcnt++;
	return 0;
}

static int
select_handler( select_context_t *ctp, int fd,
             unsigned flags, void *handle ) {

	messip_channel_t *ch = (messip_channel_t *) handle;
	ch->selcnt++;
	return 0;
}
#endif /* USE_QNXMSG */

static messip_channel_t *
messip_channel_create0( messip_cnx_t * cnx,
   const char *name,
   int32_t msec_timeout,
   int32_t maxnb_msg_buffered )
{
	int status;
	ssize_t dcount;
	int sockfd;
	struct sockaddr_in server_addr;
	struct sockaddr_in sock_name;
	socklen_t sock_namelen;
	messip_channel_t *ch;
	fd_set ready;
	struct timeval tv;
	int k;
	int32_t op;
	messip_send_channel_create_t msgsend;
	messip_reply_channel_create_t reply;
	struct iovec iovec[2];
	int flag = 0;
#ifdef USE_QNXMSG
	char namepath[PATH_MAX + NAME_MAX + 1] = "/dev/";
#endif /* USE_QNXMSG */

	if (cnx == NULL) {
		(void) pthread_once(&messip_cnx_is_initialized, messip_cnx_init);
		cnx = messip_cnx_private;
	}

	if (!cnx) {
		return NULL;
	}

	/*--- Create socket ---*/
	sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if ( sockfd < 0 )
	{
		perror("socket()");
		return NULL;
	}
	fcntl( sockfd, F_SETFL, FD_CLOEXEC );
//	logg( NULL, "%s:, sockfd=%d\n", __FUNCTION__, sockfd );

	/* Disable the Nagle (TCP No Delay) algorithm */
	if (setsockopt( sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int) ) == -1) {
	  perror("setsockopt(TCP_NODELAY)");
	}

	/*--- Bind the socket ---*/
	memset( &server_addr, 0, sizeof( server_addr ) );
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl( INADDR_ANY );
	server_addr.sin_port = htons( 0 );
	status =
	   bind( sockfd, ( struct sockaddr * ) &server_addr, sizeof( struct sockaddr_in ) );
	if ( status < 0 )
	{
		perror("bind()");
		close( sockfd );
		return NULL;
	}

	sock_namelen = sizeof( struct sockaddr_in );
	getsockname( sockfd, ( struct sockaddr * ) &sock_name, &sock_namelen );
	server_addr.sin_port = ntohs( sock_name.sin_port );

	listen( sockfd, 8 );

	/*--- Ready to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( cnx->sockfd+1, NULL, &ready, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			close( sockfd );
			errno = ETIMEDOUT;
			return NULL;
		}
	}							// if

	/*--- Send a message to the server ---*/
	op = messip_int_little_endian( MESSIP_OP_CHANNEL_CREATE );
	iovec[0].iov_base = &op;
	iovec[0].iov_len  = sizeof( int32_t );
	memset(&msgsend, 0, sizeof(msgsend));
	msgsend.pid = cnx->remote_pid;
	msgsend.tid = cnx->remote_tid;
	msgsend.maxnb_msg_buffered = maxnb_msg_buffered;
	strncpy( msgsend.channel_name, name, MESSIP_CHANNEL_NAME_MAXLEN );
	msgsend.channel_name[ MESSIP_CHANNEL_NAME_MAXLEN ] = 0;
#if USE_QNXMSG
	{
		char qnxnode[100];
		netmgr_ndtostr(ND2S_LOCAL_STR, ND_LOCAL_NODE, qnxnode, sizeof(qnxnode));
		strncpy( msgsend.qnxnode_name, qnxnode, MESSIP_QNXNODE_NAME_MAXLEN );
		msgsend.qnxnode_name[ MESSIP_QNXNODE_NAME_MAXLEN ] = 0;
	}
#else
	msgsend.qnxnode_name[0] = 0;
#endif
	msgsend.sin_port = server_addr.sin_port;
	strcpy( msgsend.sin_addr_str, inet_ntoa( server_addr.sin_addr ) );
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	status = messip_writev( cnx->sockfd, iovec, 2 );
	LIBTRACE( ( "@channel_create: send status= %d remote_socket=%d  port=%d\n",
		  status, sockfd, server_addr.sin_port ) );
	assert( ( status == sizeof( int32_t ) + sizeof( messip_send_channel_create_t ) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( cnx->sockfd+1, &ready, NULL, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			close( sockfd );
			errno = ETIMEDOUT;
			return NULL;
		}
	}							// if

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = messip_readv( cnx->sockfd, iovec, 1 );
	LIBTRACE( ( "@channel_create: reply status= %d \n", status ) );
	assert( dcount == sizeof( messip_reply_channel_create_t ) );

	/*--- Channel creation failed ? ---*/
	if ( reply.ok == MESSIP_NOK )
	{
		close( sockfd );
		errno = EEXIST;
		return NULL;
	}

	/*--- Ok ---*/
	ch = (messip_channel_t *)malloc( sizeof( messip_channel_t ) );
	strcpy( ch->name, name );
	ch->cnx = cnx;
	ch->remote_pid = cnx->remote_pid;
	ch->remote_tid = cnx->remote_tid;
	get_taskname( cnx->remote_pid, ch->remote_taskname );
	ch->sin_port = reply.sin_port;
	ch->sin_addr = reply.sin_addr;
	strcpy( ch->sin_addr_str, reply.sin_addr_str );
	ch->recv_sockfd_sz = 0;
	ch->recv_sockfd[ch->recv_sockfd_sz++] = sockfd;
	ch->send_sockfd = -1;
	ch->nb_replies_pending = 0;
	ch->new_sockfd_sz = 1;
	ch->new_sockfd = (int *) malloc( sizeof( int ) * ch->new_sockfd_sz );
	ch->channel_type =(int *) malloc( sizeof( int ) * ch->new_sockfd_sz );
	ch->receive_allmsg = (void **) malloc( sizeof(void *) * ch->new_sockfd_sz );
	ch->receive_allmsg_sz = (int *) malloc( sizeof(int) * ch->new_sockfd_sz );

#ifdef USE_SRRMOD
	if (use_srrmod) {
	    ch->srr_name_id = SrrNameAttach(0, name);
	    if (ch->srr_name_id == -1) {
		perror("SrrNameAttach()");
	    }
	}
#endif /* USE_SRRMOD */

#ifdef USE_QNXMSG
	ch->dpp = dispatch_create();
	if (ch->dpp == NULL) {
		perror("dispatch_create()");
	}

	memset( &ch->resmgr_attr, 0, sizeof( resmgr_attr_t ) );
	ch->resmgr_attr.nparts_max = 1;
	ch->resmgr_attr.msg_max_size = 2048;

	/* Setup the default I/O functions to handle open/read/write/... */
	iofunc_func_init( _RESMGR_CONNECT_NFUNCS, &ch->ConnectFuncs, _RESMGR_IO_NFUNCS, &ch->IoFuncs );

	/* Setup the attribute for the entry in the filesystem */
	iofunc_attr_init( &ch->IoFuncAttr, S_IFNAM | 0666, 0, 0 );

	strcat(namepath, name);
	ch->resmgr_id = resmgr_attach( ch->dpp, &ch->resmgr_attr, namepath, _FTYPE_ANY,
                               0, &ch->ConnectFuncs, &ch->IoFuncs, &ch->IoFuncAttr );
	if( ch->resmgr_id == -1 ) {
		perror("resmgr_attach()");
	}

	/* Setup our message callback */
	memset( &ch->message_attr, 0, sizeof( message_attr_t ) );
	ch->message_attr.nparts_max = 1;
	ch->message_attr.msg_max_size = 4096;

	/* Attach a callback (handler) for two message types */
	if (message_attach( ch->dpp, &ch->message_attr,
                                 _IO_MAX + 1, _IO_MAX + 2, message_handler, ch )) {
        perror("message_attach()");
	}

	if (pulse_attach( ch->dpp, 0, _PULSE_CODE_DISCONNECT,
						&message_handler, ch) != _PULSE_CODE_DISCONNECT) {
		perror("pulse_attach()");
	}
/*
	printf("select_attach(%d) @ %d\n", ch->recv_sockfd[0], __LINE__);
	if (select_attach(ch->dpp, NULL, ch->recv_sockfd[0],
						SELECT_FLAG_READ | SELECT_FLAG_REARM, &select_handler, ch) == -1) {
		perror("select_attach()");
	}
*/
#endif /* USE_QNXMSG */

	for ( k = 0; k < ch->new_sockfd_sz; k++ )
	{
		ch->new_sockfd[k] = -1;
		ch->channel_type[k] = -1;
		ch->receive_allmsg[k] = NULL;
		ch->receive_allmsg_sz[k] = 0;
	}

	return ch;
}								// messip_channel_create

messip_channel_t *
messip_channel_create( messip_cnx_t * cnx,
   const char *name,
   int32_t msec_timeout,
   int32_t maxnb_msg_buffered )
{
	messip_channel_t *ret;
	pthread_mutex_lock(&messip_cnx_mutex);
	ret = messip_channel_create0(cnx, name, msec_timeout, maxnb_msg_buffered);
	pthread_mutex_unlock(&messip_cnx_mutex);
	return ret;
}


static int
messip_channel_delete0( messip_channel_t * ch,
   int32_t msec_timeout )
{
	int status;
	ssize_t dcount;
	fd_set ready;
	struct timeval tv;
	int32_t op;
	messip_send_channel_delete_t msgsend;
	messip_reply_channel_delete_t reply;
	struct iovec iovec[2];

	/*--- Ready to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( ch->cnx->sockfd+1, NULL, &ready, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return -1;
		}
	}							// if

	/*--- Send a message to the server ---*/
	op = messip_int_little_endian( MESSIP_OP_CHANNEL_DELETE );
	iovec[0].iov_base = &op;
	iovec[0].iov_len = sizeof( int32_t );
	memset(&msgsend, 0, sizeof(msgsend));
	msgsend.pid = getpid(  );
	msgsend.tid = pthread_self(  );
	strcpy( msgsend.name, ch->name );
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	status = messip_writev( ch->cnx->sockfd, iovec, 2 );
	LIBTRACE( ( "@channel_delete: send status= %d\n", status ) );
	assert( ( status == sizeof( int32_t ) + sizeof( messip_send_channel_delete_t ) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( ch->cnx->sockfd+1, &ready, NULL, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return -1;
		}
	}							// if

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = messip_readv( ch->cnx->sockfd, iovec, 1 );
	LIBTRACE( ( "@channel_delete: reply status= %d \n", dcount ) );
	assert( dcount == sizeof( messip_reply_channel_delete_t ) );

	while(ch->recv_sockfd_sz) {
#ifdef USE_QNXMSG
/*
		printf("select_detach(%d) @ %d\n", ch->recv_sockfd[ch->recv_sockfd_sz], __LINE__);
		if (select_detach(ch->dpp, ch->recv_sockfd[ch->recv_sockfd_sz])) {
			perror("select_detach()");
		}
*/
#endif /* USE_QNXMSG */
		shutdown(ch->recv_sockfd[ch->recv_sockfd_sz-1], SHUT_RDWR);
		close(ch->recv_sockfd[ch->recv_sockfd_sz-1]);
		ch->recv_sockfd_sz--;
	}

#ifdef USE_SRRMOD
	if (use_srrmod && ch->srr_name_id >= 0) {
	    if (SrrNameDetach(0, ch->srr_name_id) == -1) {
		perror("SrrNameDetach()");
	    }
	}
#endif /* USE_SRRMOD */

#ifdef USE_QNXMSG
	if (pulse_detach( ch->dpp,  _PULSE_CODE_DISCONNECT, 0)) {
		perror("pulse_detach()");
	}

	if (message_detach(ch->dpp, _IO_MAX + 1, _IO_MAX, 0)) {
		perror("message_detach()");
	}
/*
	printf("select_detach(%d) @ %d\n", ch->recv_sockfd[0], __LINE__);
	if (select_detach(ch->dpp, ch->recv_sockfd[0])) {
		perror("select_detach()");
	}
*/
	if (resmgr_detach(ch->dpp, ch->resmgr_id, 0)) {
		perror("resmgr_detach()");
	}

	if(ch->dpp) {
		if (dispatch_destroy(ch->dpp)) {
			perror("dispatch_destroy()");
		}
		ch->dpp = NULL;
	}
#endif /* USE_QNXMSG */

	free(ch->new_sockfd);
	free(ch->channel_type);
	free(ch->receive_allmsg);
	free(ch->receive_allmsg_sz);
	free(ch);

	/*--- Channel creation failed ? ---*/
	return reply.nb_clients;

}								// messip_channel_delete

int
messip_channel_delete( messip_channel_t * ch,
   int32_t msec_timeout )
{
	int ret;
	pthread_mutex_lock(&messip_cnx_mutex);
	ret = messip_channel_delete0(ch, msec_timeout);
	pthread_mutex_unlock(&messip_cnx_mutex);
	return ret;
}

static messip_channel_t *
messip_channel_connect0( messip_cnx_t * cnx,
   const char *name,
   int32_t msec_timeout )
{
	int status;
	messip_channel_t *info = NULL;
	fd_set ready;
	struct timeval tv;
	messip_datasend_t datasend;
	ssize_t dcount;
	int32_t op;
	messip_send_channel_connect_t msgsend;
	messip_reply_channel_connect_t msgreply;
	struct iovec iovec[2];
	struct sockaddr_in sockaddr;
	int flag = 0;
#ifdef USE_QNXMSG
	char namepath[PATH_MAX + NAME_MAX + 1];
#endif /* USE_QNXMSG */

	if (cnx == NULL) {
		(void) pthread_once(&messip_cnx_is_initialized, messip_cnx_init);
		cnx = messip_cnx_private;
	}

	if (!cnx) {
		return NULL;
	}

	/*--- Ready to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return NULL;
		}
	}							// if

	/*--- Send a message to the messip manager ---*/
	op = messip_int_little_endian( MESSIP_OP_CHANNEL_CONNECT );
	iovec[0].iov_base = &op;
	iovec[0].iov_len = sizeof( int32_t );
	memset(&msgsend, 0, sizeof(msgsend));
	msgsend.pid = cnx->remote_pid;
	msgsend.tid = cnx->remote_tid;
	strncpy( msgsend.name, name, MESSIP_CHANNEL_NAME_MAXLEN );
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	status = messip_writev( cnx->sockfd, iovec, 2 );
	LIBTRACE( ( "@channel_connect: send status= %d  sockfd=%d\n", status, cnx->sockfd ) );
	if( ( status != sizeof( int32_t ) + sizeof( messip_send_channel_connect_t ) ) )
		fprintf( stderr, ">> %d: status=%d \n", getpid(), status );
	assert( ( status == sizeof( int32_t ) + sizeof( messip_send_channel_connect_t ) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, &ready, NULL, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return NULL;
		}
	}							// if

	/*--- Now wait for an answer from the messip manager ---*/
	for (;;)
	{
		iovec[0].iov_base = &msgreply;
		iovec[0].iov_len  = sizeof( msgreply );
		dcount = messip_readv( cnx->sockfd, iovec, 1 );
		if ( ( dcount == -1 ) && ( errno == EINTR ) )
			continue;
		break;
	}							// for (;;)
	LIBTRACE( ( "@channel_connect: reply dcount=%d already_connected=%d\n", dcount,
		  msgreply.f_already_connected ) );
	if ( dcount != sizeof( messip_reply_channel_connect_t ) )
		fprintf( stderr, "dcount=%d errno=%d\n", dcount, errno );
	assert( dcount == sizeof( messip_reply_channel_connect_t ) );

	/*--- Locate channel has failed ? ---*/
	if ( msgreply.ok == MESSIP_NOK )
	{
//		fprintf(stderr, "Locate channel has failed: %s\n", name);
		return NULL;
	}

	/*--- Use an existant connection or create a new one ---*/
	if ( msgreply.f_already_connected )
	{
		int n,
		  found;

		for ( found = 0, n = 0; n < nb_list_connect; n++ )
		{
			if ( !strcmp( list_connect[n].name, name ) )
			{
				found = 1;
				break;
			}
		}
		assert( found == 1 );
		info = list_connect[n].info;
		info->f_already_connected++;

	}
	else
	{

		/*--- Ok ---*/
		if (!info) {
		    info = (messip_channel_t *) malloc( sizeof( messip_channel_t ) );
		}

		info->f_already_connected = 0;
		pthread_mutex_init(&info->send_mutex, NULL);
		info->cnx = cnx;
		info->mgr_sockfd = msgreply.mgr_sockfd;
		get_taskname( msgreply.pid, info->remote_taskname );

		info->send_sockfd = -1;

#ifdef USE_SRRMOD
		if (use_srrmod) {
			info->srr_pid = SrrNameLocate(0, name, 0, NULL);
			if (info->srr_pid != -1) {
				info->remote_pid = info->srr_pid;
				info->remote_tid = info->srr_pid;
				info->sin_port = htons(0);
				info->sin_addr = htonl( INADDR_LOOPBACK );
				//strcpy( info->sin_addr_str, inet_ntoa(info->sin_addr) );
				strcpy( info->name, name );
				return info;
			}
		}
#endif /* USE_SRRMOD */

#ifdef USE_QNXMSG
		snprintf(namepath, sizeof(namepath)-1, "/net/%s/dev/%s",
				msgreply.qnxnode_name, name);
		info->fd = open(namepath, O_RDWR);
		if (info->fd != -1) {
			info->remote_pid = 0;
			info->remote_tid = 0;
			info->sin_port = htons(0);
			info->sin_addr = htonl( INADDR_LOOPBACK );
			//strcpy( info->sin_addr_str, inet_ntoa(info->sin_addr) );
			strcpy( info->name, name );
			return info;
		}
#endif /* USE_QNXMSG */

		info->remote_pid = msgreply.pid;
		info->remote_tid = msgreply.tid;
		info->sin_port = msgreply.sin_port;
		info->sin_addr = msgreply.sin_addr;
		strcpy( info->sin_addr_str, msgreply.sin_addr_str );
		strcpy( info->name, name );

		/*--- Create socket ---*/
		info->send_sockfd = socket( AF_INET, SOCK_STREAM, 0 );
//		logg( NULL, "%s: send_sockfd = %d \n", __FUNCTION__, info->send_sockfd );
		if ( info->send_sockfd < 0 )
		{
			perror("socket()");
			free(info);
			return NULL;
		}
		fcntl( info->send_sockfd, F_SETFL, FD_CLOEXEC );

		/* Disable the Nagle (TCP No Delay) algorithm */
		if (setsockopt( info->send_sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int) ) == -1) {
		  perror("setsockopt(TCP_NODELAY)");
		}

		/*--- Connect socket using name specified ---*/
		memset( &sockaddr, 0, sizeof( sockaddr ) );
		sockaddr.sin_family = AF_INET;
		sockaddr.sin_port = htons( info->sin_port );
		sockaddr.sin_addr.s_addr = info->sin_addr;
		if ( connect( info->send_sockfd,
			  ( const struct sockaddr * ) &sockaddr, sizeof( sockaddr ) ) < 0 )
		{
		    perror("connect()");
			fprintf( stderr, "%s %d:\n\tUnable to connect to host %s, port %d (name=%s)\n",
			   __FILE__, __LINE__, inet_ntoa( sockaddr.sin_addr ), info->sin_port, name );
			close( info->send_sockfd );
			free(info);
			return NULL;
		}

		/*--- Update list of connections to channels ---*/
		list_connect = (list_connect_t *)
		   realloc( list_connect, sizeof( list_connect_t ) * ( nb_list_connect + 1 ) );
		nb_list_connect++;
		strcpy( list_connect[nb_list_connect - 1].name, name );
		list_connect[nb_list_connect - 1].info = info;
	}							// else

	/*--- Send a fake message ---*/
	memset(&datasend, 0, sizeof(datasend));
	datasend.flag = MESSIP_FLAG_CONNECTING;
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	dcount = messip_writev( info->send_sockfd, iovec, 1 );
	assert( dcount == sizeof( messip_datasend_t ) );

	return info;

}								// messip_channel_connect0

messip_channel_t *
messip_channel_connect( messip_cnx_t * cnx,
   const char *name,
   int32_t msec_timeout )
{
	messip_channel_t *ret;
	pthread_mutex_lock(&messip_cnx_mutex);
	ret = messip_channel_connect0(cnx, name, msec_timeout);
	pthread_mutex_unlock(&messip_cnx_mutex);
	return ret;
}

static int
messip_channel_disconnect0( messip_channel_t * ch,
   int32_t msec_timeout )
{
	int status;
	fd_set ready;
	struct timeval tv;
	messip_datasend_t datasend;
	ssize_t dcount;
	struct iovec iovec[2];
	messip_send_channel_disconnect_t msgsend;
	messip_reply_channel_disconnect_t reply;
	int32_t op;
	int found, n;

	if (ch->send_sockfd != -1) {
		/*--- Timeout to write ? ---*/
		if ( msec_timeout != MESSIP_NOTIMEOUT )
		{
			FD_ZERO( &ready );
			FD_SET( ch->send_sockfd, &ready );
			tv.tv_sec = msec_timeout / 1000;
			tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
			status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
			assert( status != -1 );
			if ( !FD_ISSET( ch->send_sockfd, &ready ) )
				return MESSIP_MSG_TIMEOUT;
		}

		/*--- Message to send ---*/
		datasend.flag = MESSIP_FLAG_DISCONNECTING;
		datasend.pid = getpid(  );
		datasend.tid = pthread_self(  );
		datasend.type = -1;
		datasend.subtype = -1;
		datasend.datalen = 0;

		/*--- Send a message to the 'server' ---*/
		iovec[0].iov_base = &datasend;
		iovec[0].iov_len  = sizeof( datasend );
		dcount = messip_writev( ch->send_sockfd, iovec, 1 );
		LIBTRACE( ( "@messip_channel_disconnect: sendmsg dcount=%d local_fd=%d [errno=%d] \n",
			  dcount, ch->send_sockfd, errno ) );
		if(dcount != sizeof( messip_datasend_t )) {
			fprintf(stderr, "error disconnecting from \"%s\" channel\n", ch->name);
		}
		assert( dcount == sizeof( messip_datasend_t ) );
	}

	/*
	 * Now notify also messip_mgr
	 */

	/*--- Ready to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return -1;
		}
	}							// if

	/*--- Send a message to the messip_mgr ---*/
	op = messip_int_little_endian( MESSIP_OP_CHANNEL_DISCONNECT );
	iovec[0].iov_base = &op;
	iovec[0].iov_len = sizeof( int32_t );
	memset(&msgsend, 0, sizeof(msgsend));
	msgsend.pid = getpid(  );
	msgsend.tid = pthread_self(  );
	strcpy( msgsend.name, ch->name );
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	status = messip_writev( ch->cnx->sockfd, iovec, 2 );
	LIBTRACE( ( "@channel_disconnect: send status= %d\n", status ) );
	assert( ( status == sizeof( int32_t ) + sizeof( messip_send_channel_disconnect_t ) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, &ready, NULL, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return -1;
		}
	}							// if

	/*--- Now wait for an answer from the messip_mgr ---*/
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = messip_readv( ch->cnx->sockfd, iovec, 1 );
	LIBTRACE( ( "@channel_disconnect: reply status= %d \n", dcount ) );
	assert( dcount == sizeof( reply ) );

#ifdef USE_SRRMOD
	if (use_srrmod && ch->srr_pid != -1) {
	    	free(ch);
		return 0;
	}
#endif /* USE_SRRMOD */

#ifdef USE_QNXMSG
	if (ch->fd != -1) {
	    	if (close(ch->fd)) {
	    		perror("name_close()");
	    	}
	    	free(ch);
		return 0;
	}
#endif /* USE_QNXMSG */

	/*--- Update list of connections to channels ---*/
	for ( found = 0, n = 0; n < nb_list_connect; n++ )
	{
		if ( !strcmp( list_connect[n].name, ch->name ) )
		{
			found = 1;
			break;
		}
	}
	if ( found )
	{
		if (ch->f_already_connected == 0) {
			// destroy access mutex
			pthread_mutex_destroy(&ch->send_mutex);

			// close the connection socket
			if (ch->send_sockfd != -1) {
				if(shutdown(ch->send_sockfd, SHUT_RDWR)) {
				    perror("shutdown()");
				}
				if(close(ch->send_sockfd)) {
				    perror("close()");
				}
			}

			// deallocate the data
			free(ch);

			// update connection list
			memmove(&list_connect[n], &list_connect[n+1], sizeof( list_connect_t ) * ( nb_list_connect - n - 1 ));
			list_connect = (list_connect_t *) realloc(list_connect, sizeof( list_connect_t )  * ( nb_list_connect - 1));
			nb_list_connect--;
		} else {
			ch->f_already_connected--;
		}
	}
	else
	{
		fprintf(stderr, "messip_channel_delete: channel not found, this should not happend\n");
	}

	/*--- Channel deletion failed ? ---*/
	return reply.ok;

}								// messip_channel_disconnect

int
messip_channel_disconnect( messip_channel_t * ch,
   int32_t msec_timeout )
{
	int ret;
	pthread_mutex_lock(&messip_cnx_mutex);
	ret = messip_channel_disconnect0(ch, msec_timeout);
	pthread_mutex_unlock(&messip_cnx_mutex);
	return ret;
}


int
messip_channel_ping( messip_channel_t * ch,
   int32_t msec_timeout )
{
	ssize_t dcount;
	messip_datasend_t datasend;
	messip_datareply_t datareply;
	struct iovec iovec[1];
	fd_set ready;
	struct timeval tv;
	int status;

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->send_sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->send_sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Message to send ---*/
	datasend.flag = MESSIP_FLAG_PING;
	datasend.pid = getpid(  );
	datasend.tid = pthread_self(  );
	datasend.type = -1;
	datasend.subtype = -1;
	datasend.datalen = 0;

	/*--- Send a message to the 'server' ---*/
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	dcount = messip_writev( ch->send_sockfd, iovec, 1 );
	LIBTRACE( ( "@messip_channel_ping: sendmsg dcount=%d local_fd=%d [errno=%d] \n",
		  dcount, ch->send_sockfd, errno ) );
	assert( dcount == sizeof( messip_datasend_t ) );

	/*--- Timeout to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->send_sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, &ready, NULL, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->send_sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}
	else
	{
		FD_ZERO( &ready );
		FD_SET( ch->send_sockfd, &ready );
		status = select( ch->send_sockfd+1, &ready, NULL, NULL, NULL );
	}

	/*--- Read reply from 'server' ---*/
	iovec[0].iov_base = &datareply;
	iovec[0].iov_len  = sizeof( datareply );
	dcount = messip_readv( ch->send_sockfd, iovec, 1 );
	if ( dcount <= 0 )
	{
		LIBTRACE( ( "@%s %d\n\t dcount=%d  errno=%d\n",
			  __FILE__, __LINE__, dcount, errno ) );
		return -1;
	}

	/*--- Ok ---*/
	ch->remote_pid = datareply.pid;
	ch->remote_tid = datareply.tid;
	return 0;

}								// messip_channel_ping


static int
ping_reply( messip_channel_t * ch,
   int index,
   int32_t msec_timeout )
{
	ssize_t dcount;
	struct iovec iovec[1];
	messip_datareply_t datareply;
	fd_set ready;
	struct timeval tv;
	int status;

	/*--- Message to reply back ---*/
	datareply.pid = getpid(  );
	datareply.tid = pthread_self(  );
	datareply.datalen = 0;
	datareply.answer = -1;

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->new_sockfd[index], &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->new_sockfd[index], &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &datareply;
	iovec[0].iov_len  = sizeof( datareply );
	dcount = messip_writev( ch->new_sockfd[index], iovec, 1 );
	LIBTRACE( ( "@ping_reply: sendmsg: dcount=%d  index=%d new_sockfd=%d errno=%d\n",
		  dcount, index, ch->new_sockfd[index], errno ) );
	assert( dcount == sizeof( messip_datareply_t ) );

	/*--- Ok ---*/
	return 0;

}								// ping_reply


static int
reply_to_thread_client_send_buffered_msg( int sockfd,
   int32_t msec_timeout )
{
	ssize_t dcount;
	struct iovec iovec[1];
	messip_datareply_t datareply;
	fd_set ready;
	struct timeval tv;
	int status;

	/*--- Message to reply back ---*/
	datareply.pid = getpid(  );
	datareply.tid = pthread_self(  );
	datareply.datalen = -1;
	datareply.answer  = -1;

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( sockfd+1, NULL, &ready, NULL, &tv );	// <&>
		assert( status != -1 );
		if ( !FD_ISSET( sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &datareply;
	iovec[0].iov_len  = sizeof( datareply );
	dcount = messip_writev( sockfd, iovec, 1 );
//	logg( NULL, "@reply_to_thread_client_send_buffered_msg to sockfd=%d: sendmsg: dcount=%d  errno=%d\n",
//		  sockfd, dcount, errno );
	assert( dcount == sizeof( messip_datareply_t ) );

	/*--- Ok ---*/
	return 0;

}								// reply_to_thread_client_send_buffered_msg


#ifdef MESSIP_INFORM_STATE

static int
messip_send_inform_messipmgr( messip_channel_t * ch,
   int status,
   pid_t pid_blocked_on,
   pthread_t tid_blocked_on )
{
	ssize_t dcount;
	int32_t op;
	messip_send_inform_messipmgr_t msgsend;
	messip_reply_inform_messipmgr_t msgreply;
	struct iovec iovec[2];
	fd_set ready;
	struct timeval tv;
	int ok;

	/*--- Ready to write ? ---*/
	FD_ZERO( &ready );
	FD_SET( ch->cnx->sockfd, &ready );
	tv.tv_sec  = 0;
	tv.tv_usec = 0;
	ok = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
	assert( ok != -1 );
	if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
	{
		assert( 0 );
	}

	/*--- Send a message to the server ---*/
	op = messip_int_little_endian( MESSIP_DEBUG_OP_INFORM_STATE );
	iovec[0].iov_base = &op;
	iovec[0].iov_len  = sizeof( int32_t );
	msgsend.pid = getpid(  );
	msgsend.tid = pthread_self(  );
	msgsend.status = status;
	msgsend.pid_blocked_on  = pid_blocked_on;
	msgsend.tid_blocked_on  = tid_blocked_on;
	msgsend.when_blocked_on = time(NULL);
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	dcount = messip_writev( ch->cnx->sockfd, iovec, 2 );
	LIBTRACE( ( "@messip_send_inform_messipmgr: send dcount= %d\n", dcount ) );
	assert( dcount == sizeof( int32_t ) + sizeof( messip_send_inform_messipmgr_t ) );

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = messip_readv( ch->cnx->sockfd, iovec, 1 );
	LIBTRACE( ( "@messip_send_inform_messipmgr: reply dcount= %d \n", dcount ) );
	assert( dcount == sizeof( msgreply ) );

	/*--- Done ---*/
	return 0;

}								// messip_send_inform_messipmgr

#endif

#define MAX(a, b)	(((a) > (b)) ? (a) : (b))

int
messip_receive( messip_channel_t * ch,
   int32_t * type,
   int32_t * subtype,
   void *rec_buffer,
   int32_t maxlen,
   int32_t msec_timeout )
{
	ssize_t dcount;
	struct iovec iovec[2];
	messip_datasend_t datasend;
	int new_sockfd = -1;
	struct sockaddr_in client_addr;
	socklen_t client_addr_len;
	fd_set ready;
	struct timeval tv;
	int status;
	uint32_t len;
	int len_to_read;
	int n,
	  k,
	  nothing,
	  index;
	void *rbuff = NULL;

#ifdef MESSIP_INFORM_STATE
	/*--- Notify the MessIP manager, for DEBUG purpose only ---*/
	messip_send_inform_messipmgr(ch,
		MESSIP_STATE_RECEIVE_BLOCKED, -1, -1 );
#endif

	if ( ch->nb_replies_pending == ch->new_sockfd_sz )
	{
		ch->new_sockfd = (int *)
			realloc( ch->new_sockfd, sizeof( int ) * ( ch->new_sockfd_sz + 1 ) );
		ch->new_sockfd[ch->new_sockfd_sz] = -1;
		ch->channel_type = (int *)
			realloc( ch->channel_type, sizeof( int ) * ( ch->new_sockfd_sz + 1 ) );
		ch->receive_allmsg = (void **)
			realloc( ch->receive_allmsg, sizeof(void*) * (ch->new_sockfd_sz + 1) );
		ch->receive_allmsg_sz = (int *)
			realloc( ch->receive_allmsg_sz, sizeof(int) * (ch->new_sockfd_sz + 1) );
		ch->receive_allmsg[ch->new_sockfd_sz] = NULL;
		index = ch->new_sockfd_sz++;
	}
	else
	{
//		printf("for(index = 0; index < %d; index++)...", ch->new_sockfd_sz);
		for ( index = 0; index < ch->new_sockfd_sz; index++ )
			if ( ch->new_sockfd[index] == -1 )
				break;
//		printf("-> index = %d\n", index);
		assert( index < ch->new_sockfd_sz );
	}

  restart:

#if USE_QNXMSG

	for ( n = 0; n < ch->recv_sockfd_sz; n++ ) {
		//printf("\\select_attach[%d] @ %d\n", ch->recv_sockfd[n], __LINE__);
		if (select_attach(ch->dpp, NULL, ch->recv_sockfd[n],
							SELECT_FLAG_READ | SELECT_FLAG_REARM, &select_handler, ch) == -1) {
			perror("select_attach()");
		}
	}

	ch->ctp = dispatch_context_alloc(ch->dpp);
	if (!ch->ctp) {
		perror("dispatch_context_alloc()");
	}

	ch->msgcnt = 0;
	ch->selcnt = 0;
	do {
		//printf("dispatch_block()\n");
		ch->new_ctp = dispatch_block( ch->ctp );
		if(ch->new_ctp) {
				//printf("dispatch_handler()\n");
				dispatch_handler( ch->new_ctp );
			} else {
				perror("dispatch_block");
			}
			//printf("m: %d, s: %d\n", ch->msgcnt, ch->selcnt);
	} while(ch->msgcnt == 0 && ch->selcnt == 0);

	for ( n = 0; n < ch->recv_sockfd_sz; n++ ) {
		//printf("/select_detach[%d] @ %d\n", ch->recv_sockfd[n], __LINE__);
		if (select_detach(ch->dpp, ch->recv_sockfd[n])) {
			perror("select_detach()");
		}
	}

	if (ch->msgcnt > 0) {

		ch->remote_pid = ch->new_ctp->message_context.info.pid;
		ch->remote_tid = ch->new_ctp->message_context.info.tid;
		ch->datalen = ch->new_ctp->message_context.info.srcmsglen;
		ch->datalenr = ch->new_ctp->message_context.info.msglen;
		ch->new_sockfd[index] = ch->new_ctp->message_context.rcvid;
		ch->channel_type[index] = CHANNEL_TYPE_QNXMSG;
		ch->nb_replies_pending++;
		//printf("ch->datalen = %d\n", ch->datalen);
		memcpy(rec_buffer,
			(char *) ch->new_ctp->message_context.msg + sizeof(uint16_t),
			MAX(maxlen, ch->datalen) - sizeof(uint16_t));

		dispatch_context_free(ch->ctp);
		return index;
	}

	dispatch_context_free(ch->ctp);

#endif /* USE_QNXMSG */

	/*--- Timeout ? ---*/
	do
	{
		int maxfd = 0;
		FD_ZERO( &ready );
		for ( n = 0; n < ch->recv_sockfd_sz; n++ )
		{
//			printf("messip_receive(): FD_SET(%d)\n", ch->recv_sockfd[n]);
			FD_SET( ch->recv_sockfd[n], &ready );
			if ( ch->recv_sockfd[n] > maxfd )
				maxfd = ch->recv_sockfd[n];
		}

#ifdef USE_SRRMOD
		if (use_srrmod) {
		FD_SET (SrrFd(), &ready);
		if (ch->srr_name_id > maxfd) {
		    maxfd = ch->srr_name_id;
		}
		}
#endif /* USE_SRRMOD */

		if ( msec_timeout != MESSIP_NOTIMEOUT )
		{
			if ( msec_timeout == 1 )
			{
				// the smallest possible timeout requested
				tv.tv_sec = 0;
				tv.tv_usec = 1;
			}
			else
			{
				tv.tv_sec  = msec_timeout/1000;
				tv.tv_usec = (msec_timeout%1000) * 1000;
			}
			status = select( maxfd+1, &ready, NULL, NULL, &tv );
		}
		else
		{
		    status = select( maxfd+1, &ready, NULL, NULL, NULL );
		}
	} while ( (status == -1) && (errno == EINTR) );
	if ( status == -1 )
		return -1;
	if ( (msec_timeout != MESSIP_NOTIMEOUT) && (status == 0) )
	{
		ch->new_sockfd[index] = -1;
		*type = -1;
		*subtype = -1;
		return MESSIP_MSG_TIMEOUT;
	}

#ifdef USE_SRRMOD
	if (use_srrmod) {
		if (FD_ISSET (SrrFd(), &ready)) {
		    ch->remote_pid = SrrReceive(ch->srr_name_id, rec_buffer, (size_t *) &maxlen);
		    if (ch->remote_pid == -1) {
			return -1;
		    }
		    ch->datalen = maxlen;
		    ch->datalenr = maxlen;
		    ch->nb_replies_pending++;
		    ch->new_sockfd[index] = ch->remote_pid;
		    ch->channel_type[index] = CHANNEL_TYPE_SRRMOD;
		    return index;
		}
	}
#endif /* USE_SRRMOD */

	for ( nothing = 1, n = 0; n < ch->recv_sockfd_sz; n++ )
	{
		if ( FD_ISSET( ch->recv_sockfd[n], &ready ) )
		{
			nothing = 0;
			break;
		}
	}
	if ( nothing )
	{
		*type = -1;
		*subtype = -1;
		ch->new_sockfd[index] = -1;
		return MESSIP_MSG_TIMEOUT;
	}

	/*--- Accept a new connection ---*/
	if ( !n )
	{
		client_addr_len = sizeof( struct sockaddr_in );
		new_sockfd = accept( ch->recv_sockfd[0],
		   ( struct sockaddr * ) &client_addr, &client_addr_len );
		if ( new_sockfd == -1 )
		{
			fprintf( stderr, "messip_receive %s %d\n: %s",
			   __FILE__, __LINE__, strerror(errno));
			ch->new_sockfd[index] = new_sockfd;
			ch->channel_type[index] = CHANNEL_TYPE_MESSIP;
			return -1;
		}

//		logg( NULL,
//		   "%s: accepted a msg from %s, port=%d  - old socket=%d new=%d\n",
//		   	  __FUNCTION__,
//			  inet_ntoa( client_addr.sin_addr ), client_addr.sin_port, ch->recv_sockfd[0],
//			  new_sockfd );
#ifdef USE_QNXMSG
/*
		printf("select_attach(%d) & %d\n", new_sockfd, __LINE__);
		if (select_attach(ch->dpp, NULL, new_sockfd,
							SELECT_FLAG_READ | SELECT_FLAG_REARM, &select_handler, ch) == -1) {
			perror("select_attach()");
		}
*/
#endif /* USE_QNXMSG */
		ch->recv_sockfd[ch->recv_sockfd_sz++] = new_sockfd;
	}
	else
	{
		new_sockfd = ch->recv_sockfd[n];
	}

	/*--- Create a new channel info ---*/
	ch->new_sockfd[index] = new_sockfd;
	ch->channel_type[index] = CHANNEL_TYPE_MESSIP;
//	logg( NULL, "@messip_receive: pending=%d sz=%d new_sockfd=%d index=%d\n",
//		ch->nb_replies_pending, ch->new_sockfd_sz, new_sockfd, index );

	// record socket number (for QNX scoid compatibility)
	ch->lastmsg_sockfd = new_sockfd;

	/*--- (R1) First read the fist part of the message ---*/
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	dcount = messip_readv( new_sockfd, iovec, 1 );
	if ( ( dcount == 0 ) || ( ( dcount == -1 ) && ( errno == ECONNRESET ) ) )
	{
#ifdef USE_QNXMSG
/*
		printf("select_detach(%d) @ %d\n", ch->recv_sockfd[n], __LINE__);
		if (select_detach(ch->dpp, ch->recv_sockfd[n])) {
			perror("select_detach()");
		}
*/
#endif /* USE_QNXMSG */
//		fprintf(stderr, "actually disconnect, dcount %d ECONNRESET %d\n", dcount, ( errno == ECONNRESET ) ? 1 : 0);
		shutdown( new_sockfd, SHUT_RDWR );
		close( new_sockfd );
		for ( k = (n) ? n + 1 : ch->recv_sockfd_sz; k < ch->recv_sockfd_sz; k++ )
			ch->recv_sockfd[k - 1] = ch->recv_sockfd[k];
		ch->recv_sockfd_sz--;
		ch->new_sockfd[index] = -1;
//		goto restart;
		return MESSIP_MSG_DISCONNECT;
	}
	if ( dcount == -1 )
	{
		fprintf( stderr, "messip_receive) %s %d\n\tdcount=%d  errno=%d\n",
		   __FILE__, __LINE__, dcount, errno );
		ch->new_sockfd[index] = new_sockfd;
		return -1;
	}
	if ( datasend.flag == MESSIP_FLAG_CONNECTING ) {
//		goto restart;
		ch->new_sockfd[index] = -1;
		return MESSIP_MSG_CONNECTING;
	}
//	logg( NULL, "@messip_receive part1: dcount=%d state=%d datalen=%d flags=%d\n",
//		  dcount, datasend.state, datasend.datalen, datasend.flag );

	/*--- If message was a channel-disconnect, nothing additional to read ---*/
	ch->remote_pid = datasend.pid;
	ch->remote_tid = datasend.tid;
	if ( datasend.flag == MESSIP_FLAG_DISCONNECTING )
	{
		*type    = -1;
		*subtype = new_sockfd;
		ch->new_sockfd[index] = -1;
//		fprintf(stderr, "disconnect message\n");
//		close(new_sockfd);
//		return MESSIP_MSG_DISCONNECT;
		goto restart;
	}							// if
	if ( datasend.flag == MESSIP_FLAG_DISMISSED )
	{
		*type    = new_sockfd;
		*subtype = ch->recv_sockfd[0];
		ch->new_sockfd[index] = -1;
		return MESSIP_MSG_DISMISSED;
	}							// if
	if ( datasend.flag == MESSIP_FLAG_DEATH_PROCESS )
	{
		*type    = datasend.pid;
		*subtype = (int)datasend.tid;
		ch->new_sockfd[index] = -1;
		return MESSIP_MSG_DEATH_PROCESS;
	}							// if

	*type = datasend.type;
	*subtype = datasend.subtype;

	/*--- If message was a timer, nothing additional to read ---*/
	if ( datasend.flag == MESSIP_FLAG_TIMER )
	{
		ch->datalen  = -1;
		ch->datalenr = new_sockfd;
		ch->new_sockfd[index] = -1;
		return MESSIP_MSG_TIMER;
	}

	/*--- If message is a ping, reply to the sender ---*/
	if ( datasend.flag == MESSIP_FLAG_PING )
	{
		ping_reply( ch, index, msec_timeout );
		goto restart;
	}

	/*--- Dynamic allocation asked ? ---*/
	ch->datalen  = datasend.datalen;
	ch->datalenr = 0;
	iovec[0].iov_base = &len;
	iovec[0].iov_len  = sizeof( uint32_t );
	if ( ( rec_buffer != NULL ) && ( maxlen == 0 ) )
	{
		rbuff = malloc( datasend.datalen );
		if ( rbuff == NULL )
		{
			ch->new_sockfd[index] = -1;
			errno = ENOMEM;
			return MESSIP_NOK;
		}
		len_to_read = datasend.datalen;
		iovec[1].iov_base = rbuff;
		iovec[1].iov_len  = len_to_read;
	}
	else
	{
		len_to_read = ( maxlen < datasend.datalen ) ? maxlen : datasend.datalen;
		iovec[1].iov_base = rec_buffer;
		iovec[1].iov_len  = len_to_read;
	}

	/*--- (R2) Now read the message, unless if it's a timer ---*/
	dcount = messip_readv( new_sockfd, iovec, 2 );
//	logg( NULL, "@messip_receive part2: dcount=%d len_to_read=%d\n",
//		  dcount, len_to_read );
	if ( ( dcount == 0 ) || ( ( dcount == -1 ) && ( errno == ECONNRESET ) ) )
	{
#ifdef USE_QNXMSG
/*
		printf("select_detach(%d] @ %d\n", ch->recv_sockfd[n], __LINE__);
		if (select_detach(ch->dpp, ch->recv_sockfd[n])) {
			perror("select_detach()");
		}
*/
#endif /* USE_QNXMSG */
		shutdown( ch->recv_sockfd[n], SHUT_RDWR );
		close( ch->recv_sockfd[n] );
		for ( k = n + 1; k < ch->recv_sockfd_sz; k++ )
			ch->recv_sockfd[k - 1] = ch->recv_sockfd[k];
		ch->recv_sockfd_sz--;
		goto restart;
	}
	if ( dcount == -1 )
	{
		TRACE( "messip_receive) %s %d\n\tdcount=%d: %s\n",
			  __FILE__, __LINE__, dcount, strerror( errno ) );
		ch->new_sockfd[index] = new_sockfd;
		return -1;
	}
	assert( dcount == (ssize_t) (sizeof( uint32_t ) + len_to_read) );
	ch->datalenr = len_to_read;

	/*
		Allocate a temp buffer to hold the whole message - used by Msgread()
		Will be free-ed by Reply()
	*/
	if ( datasend.flag == MESSIP_FLAG_BUFFERED )
	{
		ch->receive_allmsg[ index ] = malloc( datasend.datalen );
		ch->receive_allmsg_sz[ index ] = datasend.datalen;
		assert( len_to_read <= datasend.datalen );
		memmove( ch->receive_allmsg[ index ], rec_buffer, len_to_read );
	}
	else
	{
		ch->receive_allmsg[ index ] = NULL;
		ch->receive_allmsg_sz[ index ] = 0;
	}

	/*
		Read more data ? (provided buffer was too small)
	*/
//	logg( NULL, "+++ datalen=%d maxlen=%d\n", datasend.datalen, maxlen );
	if ( (rec_buffer != NULL) && (maxlen != 0) && (maxlen < datasend.datalen) )
	{

		/*--- Now read the message, unless if it's a timer ---*/
		char *t = (char *)ch->receive_allmsg[ index ];
		iovec[0].iov_base = &t[ len_to_read ];
		len_to_read = ch->datalen - maxlen;
		iovec[0].iov_len  = len_to_read;
		dcount = messip_readv( new_sockfd, iovec, 1 );
		if ( dcount == -1 )
		{
			TRACE( "messip_receive_more) %s %d\n\tdcount=%d  errno=%d\n",
				  __FILE__, __LINE__, dcount, errno );
			return -1;
		}
//		fprintf( stdout, "@messip_receive part3: dcount=%d len_to_read=%d\n",
//			  dcount, len_to_read );
		assert( dcount == len_to_read );
		ch->datalenr += len_to_read;

	}

	/*--- Dynamic allocation ? ---*/
	if ( ( rec_buffer != NULL ) && ( maxlen == 0 ) )
		*( void ** ) rec_buffer = rbuff;

#ifdef MESSIP_INFORM_STATE
	/*--- Notify the MessIP manager, for DEBUG purpose only ---*/
	messip_send_inform_messipmgr(ch,
		MESSIP_STATE_NIL, -1, -1 );
#endif

	/*--- Ok ---*/
	if ( ( datasend.flag == MESSIP_FLAG_BUFFERED ) || ( datasend.flag == MESSIP_FLAG_PROXY ) )
	{
		reply_to_thread_client_send_buffered_msg( new_sockfd, msec_timeout );
//		reply_to_thread_client_send_buffered_msg( ch->cnx->sockfd, msec_timeout );
		ch->new_sockfd[index] = -1;
		if ( datasend.flag == MESSIP_FLAG_PROXY )
			ch->remote_pid = datasend.type;
		return MESSIP_MSG_NOREPLY;
	}
	else if (datasend.flag == MESSIP_FLAG_1WAY_MESSAGE)
	{
		ch->new_sockfd[index] = -1;
		ch->channel_type[index] = -1;

		/*
		if (ch->receive_allmsg[index]) {
		    free( ch->receive_allmsg[index] );
		}
		ch->receive_allmsg[index] = NULL;
		ch->receive_allmsg_sz[index] = 0;
		*/

		return MESSIP_MSG_NOREPLY;
	}
	else
	{
		ch->nb_replies_pending++;
		return index;
	}

}								// messip_receive


static int
messip_send0( messip_channel_t *ch,
   int32_t type,
   int32_t subtype,
   const void *send_buffer,
   int send_len,
   int32_t * answer,
   void *reply_buffer,
   int reply_maxlen,
   int32_t msec_timeout )
{
	ssize_t dcount;
	messip_datasend_t datasend;
	messip_datareply_t datareply;
	struct iovec iovec[3];
	fd_set ready;
	struct timeval tv;
	int status;
	uint32_t len;
	int len_to_read;
	void *rbuff = NULL;

#ifdef MESSIP_INFORM_STATE
printf( " 2) %d\n", MESSIP_STATE_SEND_BLOCKED );
	/*--- Notify the MessIP manager, for DEBUG purpose only ---*/
	messip_send_inform_messipmgr(ch,
		MESSIP_STATE_SEND_BLOCKED, ch->remote_pid, ch->remote_tid );
#endif

#ifdef USE_SRRMOD
	if (use_srrmod && ch->srr_pid >=0 ) {
	    return SrrSend(ch->srr_pid, (void *) send_buffer, reply_buffer, send_len, reply_maxlen);
	}
#endif /* USE_SRRMOD */

#ifdef USE_QNXMSG
	if (ch->fd >=0 ) {
		iov_t siov[2], riov[1];
		uint16_t msg_no = _IO_MAX + 1;

		siov[0].iov_base = &msg_no;
		siov[0].iov_len = sizeof(msg_no);
		siov[1].iov_base = (void *) send_buffer;
		siov[1].iov_len = send_len;

		riov[0].iov_base = reply_buffer;
		riov[0].iov_len = reply_maxlen;

		return MsgSendv(ch->fd, siov, 2, riov, 1);
	}
#endif /* USE_QNXMSG */

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->send_sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( ch->send_sockfd+1, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->send_sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Message to send ---*/
	memset(&datasend, 0, sizeof(datasend));
	datasend.flag = (reply_maxlen < 0) ? MESSIP_FLAG_1WAY_MESSAGE : 0;
	datasend.pid = getpid(  );
	datasend.tid = pthread_self(  );
	datasend.type = type;
	datasend.subtype = subtype;
	datasend.datalen = send_len;

	/*--- (S1) Send a message to the 'server' ---*/
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	len = reply_maxlen;
	iovec[1].iov_base = &len;
	iovec[1].iov_len  = sizeof( uint32_t );
	iovec[2].iov_base = (void *) send_buffer;
	iovec[2].iov_len  = send_len;
	dcount = messip_writev( ch->send_sockfd, iovec, 3 );
//	logg( NULL, "{messip_send/3} sendmsg send_len=%d dcount=%d local_fd=%d [errno=%d] \n",
//		  send_len, dcount, ch->send_sockfd, errno );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d:\n\terrno=%m\n", __FILE__, __LINE__ );
		return -1;
	}
	assert( dcount == (ssize_t) ( sizeof( messip_datasend_t ) + sizeof( uint32_t ) + send_len ) );

	/* nonblocking 1way send */
	if (reply_maxlen < 0) {
		/*--- Ok ---*/
		ch->datalen    = 0;
		ch->datalenr   = 0;
		return 0;
	}

	/*--- Timeout to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->send_sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( ch->send_sockfd+1, &ready, NULL, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->send_sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- (S2) Read reply from 'server' ---*/
	iovec[0].iov_base = &datareply;
	iovec[0].iov_len  = sizeof( datareply );
	dcount = messip_readv( ch->send_sockfd, iovec, 1 );
	if ( dcount == 0 )
	{
//      fprintf( stderr, "%s %d:\n\terrno=%d\n", __FILE__, __LINE__, errno );
		errno = ECONNRESET;
		return -1;
	}
	if ( dcount == -1 )
	{
		if ( errno != ECONNRESET )
			fprintf( stderr, "%s %d:\n\terrno=%d\n", __FILE__, __LINE__, errno );
		return -1;
	}
	*answer = datareply.answer;

	/*--- (S3) Read now the reply, if there is one ---*/
//	logg( NULL, "--- datalen=%d maxlen=%d\n", datareply.datalen, reply_maxlen );
	if ( (reply_buffer != NULL) && (reply_maxlen == 0) && (datareply.datalen > 0) )
	{
		len_to_read = datareply.datalen;
		rbuff = malloc( datareply.datalen );
		iovec[0].iov_base = rbuff;
		iovec[0].iov_len  = len_to_read;
	}
	else
	{
		len_to_read =
		   ( datareply.datalen < reply_maxlen ) ? datareply.datalen : reply_maxlen;
		iovec[0].iov_base = reply_buffer;
		iovec[0].iov_len  = len_to_read;
	}
	if ( len_to_read > 0 )
	{
		dcount = messip_readv( ch->send_sockfd, iovec, 1 );
		LIBTRACE( ( "@messip_send: recvmsg dcount=%d local_fd=%d [errno=%d]\n",
			  dcount, ch->send_sockfd, errno ) );
		if ( dcount == 0 )
		{
			fprintf( stderr, "%s %d:\n\terrno=%d\n", __FILE__, __LINE__, errno );
			errno = ECONNRESET;
			return -1;
		}
		if ( dcount == -1 )
		{
			fprintf( stderr, "%s %d:\n\terrno=%d\n", __FILE__, __LINE__, errno );
			return -1;
		}
	}							// if

	if ( len_to_read < datareply.datalen )
	{
//		logg( NULL, "ZUT!! %d %d\n", len_to_read, datareply.datalen );
		int rem = datareply.datalen - len_to_read;
		char *temp = (char *) malloc( rem );
		iovec[0].iov_base = temp;
		iovec[0].iov_len  = rem;
		dcount = messip_readv( ch->send_sockfd, iovec, 1 );
//		logg( NULL, "@messip_send: rem=%d dcount=%d local_fd=%d [errno=%d]\n",
//			  rem, dcount, ch->send_sockfd, errno );
		free( temp );
	}

	/*--- Dynamic allocation ? ---*/
	if ( ( reply_buffer != NULL ) && ( reply_maxlen == 0 ) )
		*( void ** ) reply_buffer = rbuff;

#ifdef MESSIP_INFORM_STATE
	/*--- Notify the MessIP manager, for DEBUG purpose only ---*/
	messip_send_inform_messipmgr(ch,
		MESSIP_STATE_REPLY_BLOCKED, ch->remote_pid, ch->remote_tid );
#endif

	/*--- Ok ---*/
	ch->datalen    = datareply.datalen;
	ch->datalenr   = len_to_read;
	ch->remote_pid = datareply.pid;
	ch->remote_tid = datareply.tid;
	return 0;

}								// messip_send

int
messip_send( messip_channel_t *ch,
   int32_t type,
   int32_t subtype,
   const void *send_buffer,
   int send_len,
   int32_t * answer,
   void *reply_buffer,
   int reply_maxlen,
   int32_t msec_timeout )
{
	int ret;
	pthread_mutex_lock(&ch->send_mutex);
	ret = messip_send0(ch, type, subtype, send_buffer, send_len, answer, reply_buffer, reply_maxlen, msec_timeout);
	pthread_mutex_unlock(&ch->send_mutex);
	return ret;
}

int32_t
messip_buffered_send( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   void *send_buffer,
   int send_len,
   int32_t msec_timeout )
{
	ssize_t dcount;
	fd_set ready;
	struct timeval tv;
	int status;
	int32_t op;
	messip_send_buffered_send_t msgsend;
	messip_reply_buffered_send_t msgreply;
	struct iovec iovec[3];

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Send a service message to the server + the private message ---*/
	op = messip_int_little_endian( MESSIP_OP_BUFFERED_SEND );
	iovec[0].iov_base = &op;
	iovec[0].iov_len  = sizeof( int32_t );
	msgsend.pid_from  = getpid(  );
	msgsend.tid_from  = pthread_self(  );
	msgsend.type = type;
	msgsend.subtype = subtype;
	msgsend.datalen = send_len;
	msgsend.mgr_sockfd = ch->mgr_sockfd;			// Socket in the messip_mgr

	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	iovec[2].iov_base = send_buffer;
	iovec[2].iov_len  = send_len;
	dcount = messip_writev( ch->cnx->sockfd, iovec, 3 );
//	LIBTRACE( ( "@messip_buffered_send: send status= %d  sockfd=%d\n",
//		  dcount, ch->cnx->sockfd ) );
	assert( ( dcount == (ssize_t) (sizeof( int32_t ) + sizeof( msgsend ) + send_len) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, &ready, NULL, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return -1;
		}
	}							// if

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = messip_readv( ch->cnx->sockfd, iovec, 1 );
//	printf( "@messip_buffered_send: reply dcount= %d,%d \n",
//		dcount, sizeof( msgreply ) );
	assert( dcount == sizeof( msgreply ) );

	return msgreply.nb_msg_buffered;

}								// messip_buffered_send


int
messip_reply( messip_channel_t * ch,
   int index,
   int32_t answer,
   const void *reply_buffer,
   int reply_len,
   int32_t msec_timeout )
{
	ssize_t dcount;
	struct iovec iovec[2];
	messip_datareply_t datareply;
	fd_set ready;
	struct timeval tv;
	int ret;
	int status, sz;

	if ( ( index < 0 ) || ( index > ch->nb_replies_pending ) )
		return -1;

#ifdef MESSIP_INFORM_STATE
	/*--- Notify the MessIP manager, for DEBUG purpose only ---*/
	messip_send_inform_messipmgr( ch,
		MESSIP_STATE_NIL, -1, -1 );
#endif

	switch(ch->channel_type[index]) {
#ifdef USE_SRRMOD
	    case CHANNEL_TYPE_SRRMOD:
			ret = SrrReply(ch->new_sockfd[index], (void *) reply_buffer, reply_len);
			break;
#endif /* USE_SRRMOD */
#ifdef USE_QNXMSG
	    case CHANNEL_TYPE_QNXMSG:
			ret = MsgReply(ch->new_sockfd[index], EOK, reply_buffer, reply_len);
			break;
#endif /* USE_QNXMSG */
	    case CHANNEL_TYPE_MESSIP:
			/*--- Message to reply back ---*/
			//	logg( NULL, "messip_reply:  pthread_self=%d\n", pthread_self() );
			datareply.pid = getpid(  );
			datareply.tid = pthread_self(  );
			datareply.datalen = reply_len;
			datareply.answer  = answer;

			/*--- Timeout to write ? ---*/
			if ( msec_timeout != MESSIP_NOTIMEOUT )
			{
				FD_ZERO( &ready );
				FD_SET( ch->new_sockfd[index], &ready );
				tv.tv_sec = msec_timeout / 1000;
				tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
				status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
				assert( status != -1 );
				if ( !FD_ISSET( ch->new_sockfd[index], &ready ) )
					return MESSIP_MSG_TIMEOUT;
			}

			/*--- Now wait for an answer from the server ---*/
			sz = 0;
			iovec[sz].iov_base  = &datareply;
			iovec[sz++].iov_len = sizeof( messip_datareply_t );
			if ( reply_len > 0 )
			{
				iovec[sz].iov_base  = (void *) reply_buffer;
				iovec[sz++].iov_len = reply_len;
			}
			dcount = messip_writev( ch->new_sockfd[index], iovec, sz );
			LIBTRACE( ( "@messip_reply: sendmsg: dcount=%d  index=%d new_sockfd=%d errno=%d\n",
				  dcount, index, ch->new_sockfd[index], errno ) );
			assert( dcount == (ssize_t) ( sizeof( messip_datareply_t ) + reply_len ) );
			ret = 0;
			break;
	    default:
			fprintf(stderr, "unknown channel type %d at index %d\n", ch->channel_type[index], index);
#ifdef __gnu_linux__
	    	{
			void * array[25];
			int nSize = backtrace(array, 25);
			char ** symbols = backtrace_symbols(array, nSize);
			int i;

			for (i = 0; i < nSize; i++)
			{
				fprintf(stderr, "%s\n", symbols[i]);
			}

			free(symbols);
	    	}
#endif /* __gnu_linux__ */
			assert(0);
			ret = -1;
	} /* switch */

	--ch->nb_replies_pending;
	ch->new_sockfd[index] = -1;
	ch->channel_type[index] = -1;

	if (ch->receive_allmsg[index]) {
	    free( ch->receive_allmsg[index] );
	}
	ch->receive_allmsg[index] = NULL;
	ch->receive_allmsg_sz[index] = 0;

	/*--- Ok ---*/
	return ret;

}								// messip_reply


#if !defined(__FreeBSD__) && !defined(__APPLE__) && !defined(__MACH__)

typedef struct
{
	timer_t timer_id;
	messip_channel_t *ch;
	int32_t user_type,
	user_subtype;
}
messip_timer_t;


#if defined(TIMER_USE_SIGEV_THREAD)

static int
messip_timer_send( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   int32_t msec_timeout )
{
	ssize_t dcount;
	messip_datasend_t datasend;
	struct iovec iovec[1];
	fd_set ready;
	struct timeval tv;
	int status;

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( ch->send_sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		status = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( status != -1 );
		if ( !FD_ISSET( ch->send_sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Message to send ---*/
	datasend.flag = MESSIP_FLAG_TIMER;
	datasend.pid = ch->remote_pid;
	datasend.tid = ch->remote_tid;
	datasend.type = type;
	datasend.subtype = subtype;
	datasend.datalen = 0;

	/*--- Send a message to the 'server' ---*/
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	dcount = messip_writev( ch->send_sockfd, iovec, 1 );
	LIBTRACE( ( "@messip_timer_send: sendmsg dcount=%d type=%d,%d local_fd=%d [errno=%d] \n",
		  dcount, ch->send_sockfd, type, subtype, errno ) );
	assert( dcount == sizeof( messip_datasend_t ) );

	/*--- Ok ---*/
	return 0;

} // messip_timer_send



static void
timer_thread_handler( sigval_t sigval )
{
	messip_timer_t *timer_info = ( messip_timer_t * ) sigval.SIGVAL_PTR;
	messip_channel_t *ch = ( messip_channel_t * ) timer_info->ch;
	int status;

	if ( ch->send_sockfd == -1 )
	{
		if ( ( ch =
			  messip_channel_connect( ch->cnx, ch->name, MESSIP_NOTIMEOUT ) ) == NULL )
			assert( 0 );
		timer_info->ch = ch;
	}							// if
	status = messip_timer_send( ch,
	   timer_info->user_type, timer_info->user_subtype, MESSIP_NOTIMEOUT );
}								// timer_thread_handler

#elif defined(TIMER_USE_SIGEV_SIGNAL)

static void
sig_action( int signo,
   siginfo_t *info,
   void *context )
{
	messip_timer_t *timer_info = ( messip_timer_t * ) info->si_value.SIGVAL_PTR;
	messip_channel_t *ch = ( messip_channel_t * ) timer_info->ch;
	int status;

	if ( ch->send_sockfd == -1 )
	{
		if ( ( ch =
			  messip_channel_connect( ch->cnx, ch->name, MESSIP_NOTIMEOUT ) ) == NULL )
			assert( 0 );
		timer_info->ch = ch;
	}							// if
	status = messip_timer_send( ch,
	   timer_info->user_type, timer_info->user_subtype, MESSIP_NOTIMEOUT );

}								// sig_action
#endif

timer_t messip_timer_create( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   int32_t msec_1st_shot,
   int32_t msec_rep_shot,
   int32_t msec_timeout )
{
	messip_timer_t *timer_info;
	struct sigevent event;
	struct itimerspec itime;
#if defined(TIMER_USE_SIGEV_SIGNAL)
	struct sigaction sig_act;
#endif

	timer_info = (messip_timer_t *) malloc( sizeof( messip_timer_t ) );
	timer_info->ch = ch;
	timer_info->user_type = type;
	timer_info->user_subtype = subtype;

	memset( &event, 0, sizeof( struct sigevent ) );
#if defined(TIMER_USE_SIGEV_THREAD)
	event.sigev_notify = SIGEV_THREAD;
	event.sigev_notify_function = &timer_thread_handler;
#elif defined(TIMER_USE_SIGEV_SIGNAL)
	event.sigev_notify = SIGEV_SIGNAL;
	event.sigev_signo  = SIGRTMIN;
#endif
	event.sigev_value.SIGVAL_PTR = timer_info;
	timer_create( CLOCK_REALTIME, &event, &timer_info->timer_id );

#if defined(TIMER_USE_SIGEV_SIGNAL)
	sigemptyset(&sig_act.sa_mask);				/* block current signal */
	sigaddset( &sig_act.sa_mask, SIGUSR1 );		/* and also these one: */
	sigaddset( &sig_act.sa_mask, SIGUSR2 );
	sigaddset( &sig_act.sa_mask, SIGALRM );
	sigaddset( &sig_act.sa_mask, SIGPIPE );
	sigaddset( &sig_act.sa_mask, SIGHUP );
	sigaddset( &sig_act.sa_mask, SIGCHLD );
	sigaddset( &sig_act.sa_mask, SIGRTMIN+1 );
    sig_act.sa_flags = SA_SIGINFO;
    sig_act.sa_sigaction = sig_action;
    sigaction( SIGRTMIN, 		  		/* Set action for SIGRTMIN      */
        &sig_act,                 		/* Action to take on signal     */
        0 );                      		/* Don't care about old actions */
#endif

	itime.it_value.tv_sec = msec_1st_shot / 1000;
	itime.it_value.tv_nsec = ( msec_1st_shot % 1000 ) * 1000000;
	itime.it_interval.tv_sec = msec_rep_shot / 1000;
	itime.it_interval.tv_nsec = ( msec_rep_shot % 1000 ) * 1000000;
	timer_settime( timer_info->timer_id, 0, &itime, NULL );

	return timer_info->timer_id;

}								// messip_timer_create

#endif


int32_t
messip_death_notify( messip_cnx_t *cnx,
   int status,
   int32_t msec_timeout )
{
	ssize_t dcount;
	fd_set ready;
	struct timeval tv;
	int ok;
	int32_t op;
	messip_send_death_notify_t msgsend;
	messip_reply_death_notify_t msgreply;
	struct iovec iovec[2];

	/*--- Timeout to write ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		ok = select( FD_SETSIZE, NULL, &ready, NULL, &tv );
		assert( ok != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
			return MESSIP_MSG_TIMEOUT;
	}

	/*--- Send a service message to the server + the private message ---*/
	op = messip_int_little_endian( MESSIP_OP_DEATH_NOTIFY );
	iovec[0].iov_base = &op;
	iovec[0].iov_len = sizeof( int32_t );
	memset(&msgsend, 0, sizeof(msgsend));
	msgsend.pid_from = getpid(  );
	msgsend.tid_from = pthread_self(  );
	msgsend.status = status;
	iovec[1].iov_base = &msgsend;
	iovec[1].iov_len  = sizeof( msgsend );
	dcount = messip_writev( cnx->sockfd, iovec, 2 );
	LIBTRACE( ( "@messip_death_notify: send status= %d  sockfd=%d\n",
		  dcount, cnx->sockfd ) );
	assert( ( dcount == sizeof( int32_t ) + sizeof( msgsend ) ) );

	/*--- Ready to read ? ---*/
	if ( msec_timeout != MESSIP_NOTIMEOUT )
	{
		FD_ZERO( &ready );
		FD_SET( cnx->sockfd, &ready );
		tv.tv_sec = msec_timeout / 1000;
		tv.tv_usec = ( msec_timeout % 1000 ) * 1000;
		ok = select( FD_SETSIZE, &ready, NULL, NULL, &tv );
		assert( ok != -1 );
		if ( !FD_ISSET( cnx->sockfd, &ready ) )
		{
			errno = ETIMEDOUT;
			return -1;
		}
	}							// if

	/*--- Now wait for an answer from the server ---*/
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = messip_readv( cnx->sockfd, iovec, 1 );
	LIBTRACE( ( "@messip_death_notify: reply dcount= %d \n",
		dcount ) );
	assert( dcount == sizeof( msgreply ) );

	return msgreply.ok;

}								// messip_death_notify




messip_dispatch_t *messip_dispatch_create(void) {
	messip_dispatch_t *dpp;

	dpp = malloc(sizeof(messip_dispatch_t));
	if(!dpp) {
		return NULL;
	}

	dpp->handlers = NULL;
	dpp->nb_handlers = 0;
	FD_ZERO(&dpp->ready);

	return dpp;
}

void messip_dispatch_delete(messip_dispatch_t *dpp) {
	free(dpp->handlers);
	free(dpp);
}

int messip_dispatch_attach(messip_dispatch_t *dpp,
		messip_channel_t * ch,
		int (*func) (messip_channel_t * ch, void * arg),
		void * arg) {

	int i;

	assert(ch);

	for (i = 0; i < dpp->nb_handlers; i++) {
		assert (dpp->handlers[i].ch != ch);
	}

	dpp->handlers = realloc(dpp->handlers, sizeof(messip_message_handler_t) * (dpp->nb_handlers+1));

	if (!dpp->handlers) {
		return -1;
	}

	dpp->handlers[dpp->nb_handlers].ch = ch;
	dpp->handlers[dpp->nb_handlers].func = func;
	dpp->handlers[dpp->nb_handlers].arg = arg;

	return dpp->nb_handlers++;
}

int messip_dispatch_block(messip_dispatch_t *dpp, int32_t msec_timeout) {
	int i, n; // iterators
	int maxfd = 0; // maximum filedescriptor number

	FD_ZERO(&dpp->ready);

	for(i = 0; i < dpp->nb_handlers; i++) {

		for (n = 0; n < dpp->handlers[i].ch->recv_sockfd_sz; n++ )
		{
			if(!FD_ISSET(dpp->handlers[i].ch->recv_sockfd[n], &dpp->ready)) {
//				printf("FD_SET(%d)\n", dpp->handlers[i].ch->recv_sockfd[n]);
				FD_SET( dpp->handlers[i].ch->recv_sockfd[n], &dpp->ready );
			}
			if ( dpp->handlers[i].ch->recv_sockfd[n] > maxfd )
				maxfd = dpp->handlers[i].ch->recv_sockfd[n];
		}

	}

//	printf("maxfd = %d\n", maxfd);

	return messip_select(maxfd+1, &dpp->ready, NULL, NULL, (msec_timeout == MESSIP_NOTIMEOUT) ? NULL : &dpp->timeout);
}

int messip_dispatch_handler(messip_dispatch_t *dpp) {
	int i, n; // interators
	int handled = 0;

	for(i = 0; i < dpp->nb_handlers; i++) {
		for (n = 0; n < dpp->handlers[i].ch->recv_sockfd_sz; n++ )
			{
				if (FD_ISSET( dpp->handlers[i].ch->recv_sockfd[n], &dpp->ready )) {
					dpp->handlers[i].func(dpp->handlers[i].ch, dpp->handlers[i].arg);
					handled++;
					FD_CLR(dpp->handlers[i].ch->recv_sockfd[n], &dpp->ready );
			}
		}
	}

	return handled;
}
