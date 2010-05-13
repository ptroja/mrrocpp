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
	messip_mgr		Olivier Singla - olivier@singla.us
					2001, 2002, 2003

	This is the server that manages the names, the buffered messages
	ans the proxies (non blocking fixed messages).
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
#include <signal.h>
#include <fcntl.h>
#include <assert.h>
#include <netinet/tcp.h>
#if defined(__linux__)
#include <endian.h>
#elif defined(__FreeBSD__)
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <sys/endian.h>
#elif defined(__QNX__)
#include <sys/param.h>
#elif defined(sun)
#else
#error Unsupported platform!
#endif

#include "messip.h"
#include "messip_private.h"

#include "messip_utils.h"

#include "logg_messip.h"

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

#define	LOCK \
	pthread_mutex_lock( &mutex )
#define	UNLOCK \
	pthread_mutex_unlock( &mutex )


/*
	Buffered message
*/
typedef struct buffered_msg_t
{
	pid_t pid_from;
	pthread_t tid_from;
	pid_t pid_to;
	pthread_t tid_to;
	int32_t type,
	  subtype;
	void *data;								   // Can be NULL
	int32_t datalen;
}
buffered_msg_t;

/*
	Proxies
*/
typedef struct proxy_t
{
	pid_t					pid_from;
	pthread_t				tid_from;
	pid_t					pid_to;
	int32_t					proxy_index;
	void					*data;				// Can be NULL
	int32_t					nbytes;				// -1 means this slot is available
	int32_t					priority;
	pthread_t				tid_client_proxies_to_trigger;
	int						process_to_trigger_sockfd;
	int32_t					nb_proxies_to_trigger;
}
proxy_t;
static int nb_proxies = 0;
static int proxies_sz = 0;					   // Current sizeo of the dynamic table
static proxy_t **proxies = NULL;			   // Dynamic table

/*
	List of active connexions (nodes)
*/
typedef struct
{
	int32_t				little_endian;			// Remote is little endian ?
	time_t				when;
	pid_t				pid;
	pthread_t			tid;
	char				process_name[MESSIP_CHANNEL_NAME_MAXLEN+1];
	struct sockaddr_in  xclient_addr;
	int					sockfd;
	int					nb_cnx_channels;
	int					*sockfd_cnx_channels;
#ifdef DEBUG
	int 				state;
	pid_t				pid_blocked_on;
	pthread_t			tid_blocked_on;
	time_t				when_blocked_on;
#endif
} connexion_t;
static int nb_connexions;
static connexion_t **connexions;

/*--- List of active channels ---*/
typedef struct /* channel_t */
{
	pid_t				pid;
	pthread_t			tid;
	connexion_t			*cnx;
	char				channel_name[MESSIP_CHANNEL_NAME_MAXLEN+1];
	char				qnxnode_name[MESSIP_QNXNODE_NAME_MAXLEN+1];
	time_t				when;							// When this channel has been created
	int					sockfd;
	in_port_t			sin_port;
	in_addr_t			sin_addr;
	char				sin_addr_str[48];
	int					f_notify_deaths;				// Send a Msg on the death of each process

	// Buffered Messages
	pthread_t			tid_client_send_buffered_msg;
	int					bufferedsend_sockfd;
	int32_t				maxnb_msg_buffered;
	int32_t				nb_msg_buffered;
	int					reply_on_release_sockfd;
	buffered_msg_t		**buffered_msg;					// Dynamic Array

	int					nb_clients;
	int					*cnx_clients;					// Dynamic Array

} channel_t;
static int nb_channels;
static channel_t **channels;				   // This is an array

static int f_bye;							   // Set to 1 when SIGINT has been applied

extern char *logg_dir;

enum
{
	KEY_CONNEXION_INDEX = 123,
	KEY_CONNEXION_PID,
	KEY_CONNEXION_PROCESS,
	KEY_CONNEXION_TID,
	KEY_CONNEXION_ADDRESS,
	KEY_CONNEXION_PORT,
	KEY_CONNEXION_SOCKET,
	KEY_CONNEXION_SINCE
};


static int
int_little_endian( const int v1 )
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
}								// int_little_endian


static int
qsort_channels( const void *c1,
   const void *c2 )
{
	const channel_t **ch1 = ( const channel_t ** ) c1;
	const channel_t **ch2 = ( const channel_t ** ) c2;

	return strcmp( ( *ch1 )->channel_name, ( *ch2 )->channel_name );

}								// qsort_channels


static int
bsearch_channels( const void *c1,
   const void *c2 )
{
	const channel_t **ch2 = ( const channel_t ** ) c2;

	return strcmp( (const char *) c1, (*ch2)->channel_name );

}								// bsearch_channels


static channel_t *
search_ch_by_sockfd( int sockfd )
{
	channel_t *ch;
	int index;

	for ( index = 0; index < nb_channels; index++ )
	{
		ch = channels[index];
		if ( ch->sockfd == sockfd )
			return ch;
	}

	return NULL;

}								// search_ch_by_sockfd


/*
	pid means the PID of the server
*/
static channel_t *
search_ch_by_pid( pid_t pid )
{
	channel_t *ch;
	int index;

	for ( index = 0; index < nb_channels; index++ )
	{
		ch = channels[index];
		if ( ch->pid == pid )
			return ch;
	}

	return NULL;

}								// search_ch_by_pid


static connexion_t *
search_cnx_by_sockfd( int sockfd )
{
	connexion_t *cnx;
	int index;

	for ( index = 0; index < nb_connexions; index++ )
	{
		cnx = connexions[index];
		if ( cnx->sockfd == sockfd )
			return cnx;
	}

	return NULL;

}								// search_cnx_by_sockfd


static int
do_writev(
	int sockfd,
	const struct iovec *iov,
	int iovcnt )
{
	int dcount;
	int cnt = -1;

	int sz = 0;
	int n;
	for ( n = 0; n < iovcnt; n++ )
		sz += iov[n].iov_len;

	errno = 0;
	for ( ;; )
	{
		cnt++;
		dcount = writev( sockfd, iov, iovcnt );
		if ( (dcount == -1) && (errno == ECONNRESET) )
			return dcount;
		if ( ( dcount == -1 ) && ( errno == EINTR ) )
			continue;
		break;
	}							// for (;;)
	if ( cnt )
		logg( LOG_MESSIP_NON_FATAL_ERROR, "%s %d: %d\n", __FILE__, __LINE__, cnt );

	return dcount;

}								// do_writev


static int
do_readv( int sockfd,
   const struct iovec *iov,
   int iovcnt )
{
	int dcount;
	int cnt = -1;

	for ( ;; )
	{
		cnt++;
		dcount = readv( sockfd, iov, iovcnt );
		if ( (dcount == -1) && (errno == ECONNRESET) )
			return dcount;
		if ( ( dcount == -1 ) && ( errno == EINTR ) )
			continue;
		break;
	}							// for (;;)
	if ( cnt )
		logg( LOG_MESSIP_NON_FATAL_ERROR, "%s %d: %d\n", __FILE__, __LINE__, cnt );

	return dcount;

}								// do_readv

#if defined(__linux__) || defined(sun)
#	define TID_DISPLAY	"%10lu"
#elif defined(__FreeBSD__)
#	define TID_DISPLAY	"%010X"
#elif defined(__QNX__)
#	define TID_DISPLAY	"%10d"
#else
#error Unsupported platform!
#endif

static void
debug_show( void )
{
	connexion_t *cnx;
	channel_t *ch;
	int index;
	char when[33];

	/*
	 * Print out the active connexions
	 */

	printf( "\n------------\n" );
	printf( "%d active connexion%s\n",
		nb_connexions, (nb_connexions > 1) ? "s" : "" );
	if ( nb_connexions > 0 )
	{
		printf( "         Pid Process             Tid Address       Port Socket Since              " );
#ifdef MESSIP_INFORM_STATE
		printf( "Status (Blocked)" );
#endif
		printf( "\n" );
		for ( index = 0; index < nb_connexions; index++ )
		{
			int k;
			cnx = connexions[index];
			strftime( when, 32, "%02d-%b-%Y %02H:%02M:%02S",
				localtime( &cnx->when ) );
			printf( "%3d:%8d %-12s " TID_DISPLAY " %-12s %5d %6d %-18s",
			   index,
			   cnx->pid, cnx->process_name, cnx->tid,
			   inet_ntoa( cnx->xclient_addr.sin_addr ),
			   cnx->xclient_addr.sin_port, cnx->sockfd, when );
			printf( " %d:", cnx->nb_cnx_channels );
			for ( k = 0; k < cnx->nb_cnx_channels; k++ )
				printf( "%s%d", (k) ? "-" : "", cnx->sockfd_cnx_channels[k] );
#ifdef MESSIP_INFORM_STATE
			switch( cnx->state )
			{
				case MESSIP_STATE_SEND_BLOCKED :
					printf( " Send on %d,%ld",
						cnx->pid_blocked_on, cnx->tid_blocked_on );
					break;
				case MESSIP_STATE_REPLY_BLOCKED :
					printf( " Reply on %d,%ld",
						cnx->pid_blocked_on, cnx->tid_blocked_on );
					break;
				case MESSIP_STATE_RECEIVE_BLOCKED :
					printf( " Received " );
					break;
			}					// switch
#endif
			printf( "\n" );
		}						// for
	}							// if

	/*
	 * Print out the active channels
	 */

	printf( "\n%d active channels\n", nb_channels );
	if ( nb_channels > 0 )
	{
		printf
		   ( "          Pid Process             Tid Address       Port Name            Socket Created            Buffered Clients\n" );
		for ( index = 0; index < nb_channels; index++ )
		{
			int k;
			char tmp[100];

			ch = channels[index];
			strftime( when, 32, "%02d-%b-%Y %02H:%02M:%02S", localtime( &ch->when ) );
			strcpy( tmp, "" );
			for ( k = 0; k < ch->nb_clients; k++ )
				sprintf( &tmp[strlen( tmp )], "-%d", ch->cnx_clients[k] );
			printf( "%c%3d:%8d %-12s " TID_DISPLAY " %-12s %5d %-16s %5d %-18s %5d/%-5d %d%s\n",
			   (ch->f_notify_deaths) ? 'D' : ' ',
			   index,
			   ch->pid, ch->channel_name, ch->tid,
			   ch->sin_addr_str,
			   ch->sin_port,
			   ch->channel_name,
			   ch->sockfd, when,
			   ch->nb_msg_buffered, ch->maxnb_msg_buffered, ch->nb_clients, tmp );
		}						// for
	}							// if

	/*
	 * Print out the defined proxies
	 */

	printf( "\n%d (%d) defined proxies\n",
		nb_proxies, proxies_sz );
	if ( nb_proxies > 0 )
	{
		printf( "    From:Pid      Tid   To:Pid Bytes Priority Pending\n" );
		for ( index = 0; index < proxies_sz; index++ )
		{
			proxy_t *p = proxies[index];

			if ( p->nbytes == -1 )	// Slot available ?
				continue;
			printf( "%2d:%9d " TID_DISPLAY " %8d %5d %8d %7d\n",
			   index,
			   p->pid_from, p->tid_from, p->pid_to,
			   p->nbytes, p->priority, p->nb_proxies_to_trigger );
		}						// for (index)
	}							// if

}								// debug_show


static void *
debug_thread( void *arg )
{
	int sig;
	sigset_t set;

	sigemptyset( &set );
	sigaddset( &set, SIGUSR2 );
	pthread_sigmask( SIG_BLOCK, &set, NULL );

	/*--- Wait until SIGUSR1 signal ---*/
	fprintf( stdout, "For debugging: kill -s SIGUSR1 %d\n", getpid(  ) );
	for ( ;; )
	{
		sigemptyset( &set );
		sigaddset( &set, SIGUSR1 );
		sigwait( &set, &sig );
		LOCK;
		debug_show(  );
		UNLOCK;
	}

	/*--- Never exit anyway ---*/
	return NULL;
}								// debug_thread


static void
http_table_column_title( char *msg,
   const char *title,
   const char *tag )
{
	strcat( msg, "      <td valign=\"top\" align=\"center\" bgcolor=\"#c0c0c0\">" );
	strcat( msg, "<a href=\"" );
	strcat( msg, tag );
	strcat( msg, "\"></big><b>" );
	strcat( msg, "</big><b>" );
	strcat( msg, title );
	strcat( msg, "      </b></big></td>\n" );
}								// http_table_column_title


static void
http_table_column_add_int( char **buff,
   int val )
{
	char tmp[ 100 ];
	sprintf( tmp, "%d", val );
	*buff = strdup( tmp );
}								// http_table_column_add_int


static void
http_table_column_add_tid( char **buff,
   pthread_t val )
{
	char tmp[ 100 ];
	sprintf( tmp, TID_DISPLAY, val );
	*buff = strdup( tmp );
}								// http_table_column_add_tid


static void
http_table_column_add_string( char **buff,
   char *val )
{
	char tmp[ 100 ];
	sprintf( tmp, "%s", val );
	*buff = strdup( tmp );
}								// http_table_column_add_string


static void
http_table_column_add( char *msg,
   char *val )
{
	sprintf( &msg[ strlen( msg ) ],
		"      <td valign=\"top\" align=\"right\" >%s<br></td>\n",
		val );
	free( val );
}								// http_table_column_add


static void
http_build_table_connexions( char *msg,
   int key )
{
	connexion_t *cnx;
	int index;
	char when[33];
	typedef struct
	{
		char *index;
		char *pid;
		char *process;
		char *tid;
		char *address;
		char *port;
		char *socket;
		char *since;
	} column_t;
	column_t *tab,
	   *p;

	/*--- Headers of the table ---*/
	strcat( msg,
		"<table cellpadding=\"2\" cellspacing=\"2\" border=\"1\" width=\"100%\">\n"
		"  <tbody>\n"
		"    <tr>\n" );
	http_table_column_title( msg, "Index", 		"/connexion/index" );
	http_table_column_title( msg, "Pid", 		"/connexion/pid" );
	http_table_column_title( msg, "Process", 	"/connexion/process" );
	http_table_column_title( msg, "Tid", 		"/connexion/tid" );
	http_table_column_title( msg, "Address", 	"/connexion/address" );
	http_table_column_title( msg, "Port", 		"/connexion/port" );
	http_table_column_title( msg, "Socket", 	"/connexion/socket" );
	http_table_column_title( msg, "Since", 		"/connexion/since" );
	strcat( msg, "    </tr>\n" );

	tab = (column_t *) malloc( sizeof( column_t ) *nb_connexions );
	for ( p = tab, index = 0; index < nb_connexions; index++, p++ )
	{
		cnx = connexions[index];
		http_table_column_add_int( &p->index, 		index );
		http_table_column_add_int( &p->pid, 		cnx->pid );
		http_table_column_add_string( &p->process, 	cnx->process_name );
		http_table_column_add_tid( &p->tid, 		cnx->tid );
		http_table_column_add_string( &p->address, 	inet_ntoa( cnx->xclient_addr.sin_addr ) );
		http_table_column_add_int( &p->port, 		cnx->xclient_addr.sin_port );
		http_table_column_add_int( &p->socket, 		cnx->sockfd );
		strftime( when, 32, "%02d-%b-%Y %02H:%02M:%02S",
			localtime( &cnx->when ) );
		http_table_column_add_string( &p->since, 	when );
	}							// http_build_table_connexions
	for ( p = tab, index = 0; index < nb_connexions; index++, p++ )
	{
		strcat( msg,
			"    <tr>\n" );
		http_table_column_add( msg, p->index );
		http_table_column_add( msg, p->pid );
		http_table_column_add( msg, p->process );
		http_table_column_add( msg, p->tid );
		http_table_column_add( msg, p->address );
		http_table_column_add( msg, p->port );
		http_table_column_add( msg, p->socket );
		http_table_column_add( msg, p->since );
		strcat( msg,
			"    </tr>\n" );
	}
	free( tab );

	/*--- End of table ---*/
	strcat( msg,
		"  </tbody>\n"
		"</table>\n" );

}								// http_build_table_connexions


static void
http_build_table_channels( char *msg )
{
	channel_t *ch;
	int index;
	char when[33];

	/*--- Headers of the table ---*/
	strcat( msg,
		"<table cellpadding=\"2\" cellspacing=\"2\" border=\"1\" width=\"100%\">\n"
		"  <tbody>\n"
		"    <tr>\n" );
	http_table_column_title( msg, "Index", 		"/channel/index" );
	http_table_column_title( msg, "Pid", 		"/channel/pid" );
	http_table_column_title( msg, "Process", 	"/channel/process" );
	http_table_column_title( msg, "Tid", 		"/channel/tid" );
	http_table_column_title( msg, "Address", 	"/channel/address" );
	http_table_column_title( msg, "Port", 		"/channel/port" );
	http_table_column_title( msg, "Name", 		"/channel/name" );
	http_table_column_title( msg, "Socket", 	"/channel/socket" );
	http_table_column_title( msg, "Created", 	"/channel/created" );
	http_table_column_title( msg, "Buffered", 	"/channel/buffered" );
	http_table_column_title( msg, "Clients", 	"/channel/clients" );
	strcat( msg, "    </tr>\n" );

	for ( index = 0; index < nb_channels; index++ )
	{
		int k;
		char tmp[100];

		ch = channels[index];
		strftime( when, 32, "%02d-%b-%Y %02H:%02M:%02S", localtime( &ch->when ) );
		strcpy( tmp, "" );
		for ( k = 0; k < ch->nb_clients; k++ )
			sprintf( &tmp[strlen( tmp )], "-%d", ch->cnx_clients[k] );
		sprintf( &msg[ strlen( msg ) ],
			"    <tr>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"	// index
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"	// pid
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%s<br>\n"	// task name
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >" TID_DISPLAY "<br>\n"	// tid
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%s<br>\n"	// sin_addr_str
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"	// sin_port
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%s<br>\n"	// name
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"	// sockfd
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%s<br>\n"	// when
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d/%d<br>\n"	// nb_msg_buffered, maxnb_msg_buffered
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d%s<br>\n"	// nb_clients, tmp
			"      </td>\n"
			"    </tr>\n",
//		printf( "%c%3d:%8d %-12s %4ld %-12s " TID_DISPLAY " %-16s %5d %-18s %5d/%-5d %d%s\n",
//		   (ch->f_notify_deaths) ? 'D' : ' ',
		   index,
		   ch->pid, ch->channel_name, ch->tid,
		   ch->sin_addr_str,
		   ch->sin_port,
		   ch->channel_name,
		   ch->sockfd, when,
		   ch->nb_msg_buffered, ch->maxnb_msg_buffered, ch->nb_clients, tmp );
	}						// for

	strcat( msg,
		"  </tbody>\n"
		"</table>\n" );

}								// http_build_table_channels


static void
http_build_table_proxies( char *msg )
{
	int index;

	/*--- Headers of the table ---*/
	strcat( msg,
		"<table cellpadding=\"2\" cellspacing=\"2\" border=\"1\" width=\"100%\">\n"
		"  <tbody>\n"
		"    <tr>\n" );
	http_table_column_title( msg, "Index", 		"/proxy/index" );
	http_table_column_title( msg, "From Pid", 	"/proxy/from_pid" );
	http_table_column_title( msg, "To Tid", 	"/proxy/to_pid" );
	http_table_column_title( msg, "Address", 	"/proxy/address" );
	http_table_column_title( msg, "Bytes", 		"/proxy/bytes" );
	http_table_column_title( msg, "Priority", 	"/proxy/priority" );
	http_table_column_title( msg, "Pending", 	"/proxy/pending" );
	strcat( msg, "    </tr>\n" );

	for ( index = 0; index < proxies_sz; index++ )
	{
		proxy_t *p = proxies[index];

		if ( p->nbytes == -1 )	// Slot available ?
			continue;
		sprintf( &msg[ strlen( msg ) ],
			"    <tr>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"
			"      </td>\n"
			"      <td valign=\"top\" align=\"right\" >%d<br>\n"
			"      </td>\n"
			"    </tr>\n",
				index,
				p->pid_from, p->pid_to,
				p->nbytes, p->priority, p->nb_proxies_to_trigger );
	}							// http_build_table_connexions

	/*--- End of table ---*/
	strcat( msg,
		"  </tbody>\n"
		"</table>\n" );

}								// http_build_table_proxies


static int
http_send_status( int sockfd,
   int version,
   int subversion,
   int key )
{
	struct iovec iovec[1];
	char *msg1,
	   *msg2;
	int dcount;

	/*--- Page header ---*/
	msg2 = (char *) malloc( 32768 );
	sprintf( msg2,
		"<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\">\n"
		"<html>\n"
		"<head>\n"
		"  <title>MessIP - Information</title>\n"
		"  <meta http-equiv=\"content-type\"\n"
		" content=\"text/html; charset=ISO-8859-1\">\n"
		"</head>\n" );
	strcat( msg2,
		"<b>MessIP</b> : Message Passing over TCP/IP<br>"
		"<a href=\"http://singla.us/messip/\">http://singla.us/messip/</a><br>" );
	sprintf( &msg2[ strlen( msg2 ) ],
		"Version <b>%d.%d.%c</b> compiled on <b>%s %s</b><br>",
			VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH + 'a' - 1,
			__DATE__, __TIME__ );
	strcat( msg2,
		"<hr width=\"100%\" size=\"2\"><br>" );

	/*--- Active connections ---*/
	sprintf( &msg2[ strlen( msg2 ) ],
		"<body text=\"#000000\" bgcolor=\"#dddddd\" link=\"#000099\" vlink=\"#990099\" alink=\"#000099\">\n"
		"%d active connection%s:<br>\n",
			nb_connexions, (nb_connexions > 1) ? "s" : "" );
	if ( nb_connexions > 0 )
		http_build_table_connexions( msg2, key );

	/*--- Active channels ---*/
	sprintf( &msg2[ strlen( msg2 ) ],
		"<p>%d active channel%s:<br>\n",
			nb_channels, (nb_channels > 1) ? "s" : "" );
	if ( nb_channels > 0 )
		http_build_table_channels( msg2 );

	/*--- Print out the defined proxies ---*/
	sprintf( &msg2[ strlen( msg2 ) ],
		"<p>%d (%d) defined proxies\n",
			nb_proxies, proxies_sz );
	if ( nb_proxies > 0 )
		http_build_table_proxies( msg2 );

	/*--- End of page ---*/
	strcat( msg2,
		"<br>\n"
		"<br>\n"
		"</body>\n"
		"</html>\n"
		"\n" );

	/*--- Header to send ---*/
	msg1 = (char *) malloc( 1024 );
	sprintf( msg1,
		"HTTP/%d.%d 200 OK\r\n"
		"Content-type: text/html\r\n"
		"Content-Length: %d\r\n"
		"\r\n",
			version, subversion,
			strlen( msg2 ) );
	iovec[0].iov_base = msg1;
	iovec[0].iov_len  = strlen( msg1 );
	dcount = do_writev( sockfd, iovec, 1 );
	free( msg1 );

	/*--- Send now the page ---*/
	iovec[0].iov_base = msg2;
	iovec[0].iov_len  = strlen( msg2 );
	dcount = do_writev( sockfd, iovec, 1 );
	free( msg2 );

	/*--- Ok ---*/
	return 0;

}								// http_send_status


typedef struct
{
	int sockfd_accept;
	struct sockaddr_in client_addr;
	socklen_t client_addr_len;
}
clientdescr_t;


static void *
thread_http_thread( void *arg )
{
	clientdescr_t *descr = ( clientdescr_t * ) arg;
	ssize_t dcount;
	char request[500];
	struct iovec iovec[1];
	int status,
	   version,
	   subversion;
	char tag[200];
	int key;

	TRACE( "thread_http_thread: pid=%d tid=" TID_DISPLAY "\n",
		getpid(  ), pthread_self(  ) );

	for (;;)
	{

		iovec[0].iov_base = request;
		iovec[0].iov_len  = sizeof( request ) - 1;
		dcount = do_readv( descr->sockfd_accept, iovec, 1 );
		if ( dcount <= 0 )
			break;

		if ( dcount >= 15 )
		{
			status = sscanf( request, "GET %s HTTP/%d.%d",
				tag, &version, &subversion );
			key = -1;
			if ( status == 3 )
			{
				if (      !strcmp( tag, "/connexion/index" ) )		key = KEY_CONNEXION_INDEX;
				else if ( !strcmp( tag, "/connexion/pid" ) )		key = KEY_CONNEXION_PID;
				else if ( !strcmp( tag, "/connexion/process" ) )	key = KEY_CONNEXION_PROCESS;
				else if ( !strcmp( tag, "/connexion/tid" ) )		key = KEY_CONNEXION_TID;
				else if ( !strcmp( tag, "/connexion/address" ) )	key = KEY_CONNEXION_ADDRESS;
				else if ( !strcmp( tag, "/connexion/port" ) )		key = KEY_CONNEXION_PORT;
				else if ( !strcmp( tag, "/connexion/socket" ) )		key = KEY_CONNEXION_SOCKET;
				else if ( !strcmp( tag, "/connexion/since" ) )		key = KEY_CONNEXION_SINCE;
				http_send_status( descr->sockfd_accept, version, subversion, key );
				shutdown( descr->sockfd_accept, SHUT_RDWR );	// Close the connection
				break;
			}
		}

		request[ dcount ] = 0;
		printf( "dcount=%d [%s]\n", dcount, request );

	}							// for (;;)

	/*--- Done ---*/
	if ( close( descr->sockfd_accept ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n", descr->sockfd_accept, strerror(errno) );
	free( descr );

	pthread_exit(NULL);

	return NULL;
}								// thread_http_thread


static void *
http_thread( void *arg )
{
	int port_http = *(int *)arg;
	sigset_t set;
	int sockfd;
	int status;
	int reuse;
	int flag = 0;
	struct sockaddr_in server_addr;

	sigemptyset( &set );
	sigaddset( &set, SIGUSR2 );
	pthread_sigmask( SIG_BLOCK, &set, NULL );

	/*--- Create socket ---*/
	sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if ( sockfd < 0 )
	{
		fprintf( stderr, "%s %d\n\tUnable to open a socket!\n", __FILE__, __LINE__ );
		exit( -1 );
	}

	reuse = 1;

	if ((setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
					(void *) &reuse, sizeof(int))) < 0) {
		fprintf( stderr, "%s %d\n\tUnable to set SO_REUSEADDR a socket!\n", __FILE__, __LINE__ );
		exit( -1 );
	}

	/* Disable the Nagle (TCP No Delay) algorithm */
	if (setsockopt( sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int) ) == -1) {
	  perror("setsockopt(TCP_NODELAY)");
	}

	/*--- Bind the socket ---*/
	memset( &server_addr, 0, sizeof( server_addr ) );
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl( INADDR_ANY );
	server_addr.sin_port = htons( port_http );
	status = bind( sockfd, ( struct sockaddr * ) &server_addr, sizeof( struct sockaddr_in ) );
	if ( status == EADDRINUSE )
	{
		fprintf( stderr, "%s %d\n\tUnable to bind, port %d is already in use\n",
			__FILE__, __LINE__,
		   port_http );
		if ( close( sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n", sockfd, strerror(errno) );
		exit( -1 );
	}
	if ( status < 0 )
	{
		fprintf( stderr, "%s %d\n\tUnable to bind - port %d - status=%d: %s\n",
		   __FILE__, __LINE__,
		   port_http,
		   status, strerror( errno ) );
		if ( close( sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n", sockfd, strerror(errno) );
		exit( -1 );
	}

	listen( sockfd, 8 );

	for (;;)
	{
		clientdescr_t *descr;
		pthread_t tid;
		pthread_attr_t attr;
		int flag = 0;

		descr = (clientdescr_t *) malloc( sizeof( clientdescr_t ) );
		descr->client_addr_len = sizeof( struct sockaddr_in );
		descr->sockfd_accept = accept( sockfd,
		   ( struct sockaddr * ) &descr->client_addr, &descr->client_addr_len );
		if ( descr->sockfd_accept == -1 )
		{
			if ( errno == EINTR )	// A signal has been applied
				continue;
			fprintf( stderr, "Socket non accepted: %s\n", strerror(errno) );
			if ( close( sockfd ) == -1 )
				fprintf( stderr, "Error while closing socket %d: %s\n", sockfd, strerror(errno) );
			exit( -1 );
		}

		/* Disable the Nagle (TCP No Delay) algorithm */
		if (setsockopt(descr->sockfd_accept, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int)) == -1) {
		  perror("setsockopt(TCP_NODELAY)");
		}

#if 1
		logg( LOG_MESSIP_DEBUG_LEVEL1, "http: accepted a msg from %s, port=%d, socket=%d\n",
			  inet_ntoa( descr->client_addr.sin_addr ),
			  descr->client_addr.sin_port, descr->sockfd_accept );
#endif
		pthread_attr_init( &attr );
		pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
		pthread_create( &tid, &attr, &thread_http_thread, descr );
	}							// for (;;)

	/*--- Never exit anyway ---*/
	if ( close( sockfd ) == -1 )
		fprintf( stderr, "Error while closing socket %d: %s\n", sockfd, strerror(errno) );

	return NULL;

}								// http_thread


static int
handle_client_connect( int sockfd,
   struct sockaddr_in *client_addr,
   connexion_t ** new_cnx )
{
	struct iovec iovec[1];
	messip_send_connect_t msg;
	messip_reply_connect_t reply;
	connexion_t *cnx;
	ssize_t dcount;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( msg ) )
	{
		fprintf( stderr, "%s %d\n\tread %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( msg ), strerror(errno) );
		return -1;
	}

	/*--- Allocate a new connexion ---*/
	cnx = (connexion_t *) malloc( sizeof( connexion_t ) );
	cnx->little_endian = msg.little_endian;
	time( &cnx->when );
#if BYTE_ORDER == BIG_ENDIAN
	cnx->pid = int_little_endian( msg.pid );
	cnx->tid = int_little_endian( msg.tid );
#elif BYTE_ORDER == LITTLE_ENDIAN
	cnx->pid = msg.pid;
	cnx->tid = msg.tid;
#else
#error
#endif
	strncpy( cnx->process_name, msg.process_name, MESSIP_MAXLEN_TASKNAME );
	cnx->process_name[ MESSIP_MAXLEN_TASKNAME ] = 0;
	printf("cnx->process_name = %s\n", cnx->process_name);
	memmove( &cnx->xclient_addr, client_addr, sizeof( struct sockaddr_in ) );
	cnx->nb_cnx_channels = 0;
	cnx->sockfd_cnx_channels = NULL;
	cnx->sockfd = sockfd;
#ifdef MESSIP_INFORM_STATE
	cnx->state = MESSIP_STATE_NIL;
#endif
	LOCK;
	if ( connexions == NULL )
		connexions = (connexion_t **) malloc( sizeof( connexion_t * ) );
	else
		connexions = (connexion_t **) realloc( connexions, ( nb_connexions + 1 ) * sizeof( connexion_t * ) );
	connexions[nb_connexions++] = cnx;
	UNLOCK;
	*new_cnx = cnx;

#if 0
	logg( LOG_MESSIP_DEBUG_LEVEL1, "handle_msg_connect: pid=%d[%X] tid=%ld ip=%s port=%d\n",
		  cnx->pid, cnx->pid, cnx->tid,
		  inet_ntoa( client_addr->sin_addr ), client_addr->sin_port );
#endif

	/*--- Reply to the client ---*/
	reply.ok = MESSIP_OK;
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( reply ) );

	return 0;

}								// handle_client_connect


static int
client_channel_create( int sockfd,
   struct sockaddr_in *client_addr,
   connexion_t ** cnx )
{
	channel_t **pch,
	 *ch;
	struct iovec iovec[1];
	messip_send_channel_create_t msg;
	messip_reply_channel_create_t reply;
	ssize_t dcount;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( msg ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( msg ), strerror(errno) );
		return -1;
	}
#if 1
	logg( LOG_MESSIP_INFORMATIVE, "channel_create: pid=%d tid=%d ip=%s port=%d name=%s\n",
		  int_little_endian( msg.pid ), int_little_endian( msg.tid ),
		  inet_ntoa( client_addr->sin_addr ), client_addr->sin_port, msg.channel_name );
#endif

	/*--- Is there any channel with this name ? ---*/
	memset(&reply, 0, sizeof(reply));
	LOCK;
	pch = (channel_t **) bsearch( msg.channel_name,
	   channels, nb_channels, sizeof( channel_t * ), bsearch_channels );
	if ( pch != NULL )
	{

		UNLOCK;
		reply.ok = MESSIP_NOK;

	}							// if
	else
	{

		/*--- Allocate a new channel ---*/
		if ( channels == NULL )
			channels = (channel_t **) malloc( sizeof( channel_t * ) );
		else
			channels = (channel_t **) realloc( channels, ( nb_channels + 1 ) * sizeof( channel_t * ) );
		ch = (channel_t *) calloc( 1, sizeof( channel_t ) );
		channels[nb_channels++] = ch;

		/*--- Create a new channel ---*/
		ch->cnx = *cnx;
		ch->when = time( NULL );
#if BYTE_ORDER == BIG_ENDIAN
		ch->pid = int_little_endian( msg.pid );
		ch->tid = int_little_endian( msg.tid );
		ch->maxnb_msg_buffered = int_little_endian( msg.maxnb_msg_buffered );
#elif BYTE_ORDER == LITTLE_ENDIAN
		ch->pid = msg.pid;
		ch->tid = msg.tid;
		ch->maxnb_msg_buffered = msg.maxnb_msg_buffered;
#else
#error
#endif
		ch->sockfd = sockfd;
		ch->tid_client_send_buffered_msg = 0;
		ch->bufferedsend_sockfd = 0;
		ch->nb_msg_buffered = 0;
		ch->buffered_msg = NULL;
		ch->nb_clients = 0;
		ch->cnx_clients = NULL;
		ch->f_notify_deaths = MESSIP_FALSE;		// Send a Msg on the death of each process
		strncpy( ch->channel_name, msg.channel_name, MESSIP_CHANNEL_NAME_MAXLEN );
		ch->channel_name[ MESSIP_CHANNEL_NAME_MAXLEN ] = 0;
		strncpy( ch->qnxnode_name, msg.qnxnode_name, MESSIP_QNXNODE_NAME_MAXLEN );
		ch->qnxnode_name[ MESSIP_QNXNODE_NAME_MAXLEN ] = 0;
		ch->sin_port = msg.sin_port;
		ch->sin_addr = client_addr->sin_addr.s_addr;
		strcpy( ch->sin_addr_str, inet_ntoa( client_addr->sin_addr ) );

		/*--- Keep the channels sorted ---*/
		qsort( channels, nb_channels, sizeof( channel_t * ), qsort_channels );
		reply.ok = MESSIP_OK;
		reply.sin_port = ch->sin_port;
		reply.sin_addr = ch->sin_addr;
		strcpy( reply.sin_addr_str, ch->sin_addr_str );

		UNLOCK;
	}							// else

	/*--- Reply to the client ---*/
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( reply ) );

	return 0;

}								// client_channel_create


static void
destroy_channel( channel_t * ch,
   int index )
{
	int k;

#if 0
	printf( "Destroy channel %d [%s]\n", index, ch->channel_name);
#endif

	if ( ch->tid_client_send_buffered_msg )
	{
		pthread_cancel( ch->tid_client_send_buffered_msg );
		if ( close( ch->bufferedsend_sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n", ch->bufferedsend_sockfd, strerror(errno) );
	}							// if

	if ( ch->nb_msg_buffered > 0 )
	{
		for ( k = 0; k < ch->nb_msg_buffered; k++ )
			free( ch->buffered_msg[k] );
		free( ch->buffered_msg );
	}							// if

	if ( index == -1 )
	{
		for ( k = 0; k < nb_channels; k++ )
		{
			if ( channels[k] == ch )
			{
				index = k;
				break;
			}
		}
		assert( index != -1 );
	}							// if

	free(channels[index]);

	for ( k = index + 1; k < nb_channels; k++ )
		channels[k - 1] = channels[k];

	nb_channels--;

}								// destroy_channel


static int
client_channel_delete( int sockfd,
   struct sockaddr_in *client_addr )
{
	channel_t **pch,
	 *ch;
	struct iovec iovec[1];
	messip_send_channel_delete_t msg;
	messip_reply_channel_delete_t reply;
	ssize_t dcount;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_channel_delete_t ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( messip_send_channel_delete_t ), strerror(errno) );
		return -1;
	}

#if 1
	printf( "channel_delete: pid=%d tid=%ld name=%s\n", msg.pid, msg.tid, msg.name );
#endif

	/*--- Search this channel name ---*/
	LOCK;
	pch = (channel_t**) bsearch( msg.name,
	   channels, nb_channels, sizeof( channel_t * ), bsearch_channels );
	ch = ( pch ) ? *pch : NULL;
	UNLOCK;

	/*--- Reply to the client ---*/
	if ( ch == NULL )
	{
		reply.nb_clients = -1;
	}
	else
	{
		LOCK;
		if ( ( ch->pid != msg.pid ) || ( ch->tid != msg.tid ) )
		{
			reply.nb_clients = -1;
		}						// if
		else
		{
			if ( ch->nb_clients > 0 )
			{
				reply.nb_clients = ch->nb_clients;
			}
			else
			{
				destroy_channel( ch, -1 );
				reply.nb_clients = 0;
			}
		}						// else
		UNLOCK;
	}							// else
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( reply ) );

	return MESSIP_OK;

}								// client_channel_delete


static int
client_channel_connect( int sockfd,
   struct sockaddr_in *client_addr )
{
	channel_t **pch,
	 *ch;
	struct iovec iovec[1];
	messip_send_channel_connect_t msg;
	messip_reply_channel_connect_t reply;
	ssize_t dcount;
	int k;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_channel_connect_t ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( messip_send_channel_connect_t ), strerror(errno) );
		return -1;
	}

#if 0
	TRACE( "channel_connect: pid=%d tid=%ld name=%s\n", msg.pid, msg.tid, msg.name );
#endif

	/*--- Search this channel name ---*/
	LOCK;
	pch = (channel_t **) bsearch( msg.name, channels, nb_channels, sizeof( channel_t * ), bsearch_channels );
	ch = ( pch ) ? *pch : NULL;
	UNLOCK;

	/*--- Reply to the client ---*/
	memset(&reply, 0, sizeof(reply));
	if ( ch == NULL )
	{
		reply.ok = MESSIP_NOK;
	}
	else
	{

		/*--- Is this client already connected ? ---*/
		for ( reply.f_already_connected = 0, k = 0; k < ch->nb_clients; k++ )
		{
			if ( ch->cnx_clients[k] == sockfd )
			{
				reply.f_already_connected = 1;
				break;
			}
		}

		reply.ok = MESSIP_OK;
		reply.pid = ch->pid;
		reply.tid = ch->tid;
		reply.sin_port = ch->sin_port;
		reply.sin_addr = ch->sin_addr;
		memmove( reply.sin_addr_str, ch->sin_addr_str, sizeof( reply.sin_addr_str ) );
		reply.mgr_sockfd = ch->sockfd;
		strcpy(reply.qnxnode_name, ch->qnxnode_name);
	}
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( reply ) );

	/*--- Maintain a list of clients connected to this connection ---*/
	if ( ch )
	{
		if ( !reply.f_already_connected )
		{
			connexion_t *cnx;

			LOCK;

			if ( ch->nb_clients == 0 )
				ch->cnx_clients = (int *) malloc( sizeof( int ) );
			else
				ch->cnx_clients = (int *)
				   realloc( ch->cnx_clients, ( ch->nb_clients + 1 ) * sizeof( int ) );
			ch->cnx_clients[ch->nb_clients++] = sockfd;

		    cnx	= search_cnx_by_sockfd( sockfd );
			if ( cnx->nb_cnx_channels == 0 )
				cnx->sockfd_cnx_channels = (int *) malloc( sizeof( int ) );
			else
				cnx->sockfd_cnx_channels = (int *) realloc( cnx->sockfd_cnx_channels,
					(cnx->nb_cnx_channels+1) * sizeof( int ) );
			cnx->sockfd_cnx_channels[ cnx->nb_cnx_channels++ ] = ch->sockfd;

			UNLOCK;

		}						// if
	}							// if

	return MESSIP_OK;

}								// client_channel_connect


static int
client_channel_disconnect( int sockfd,
   struct sockaddr_in *client_addr )
{
	channel_t **pch,
	 *ch;
	struct iovec iovec[1];
	messip_send_channel_disconnect_t msg;
	messip_reply_channel_disconnect_t reply;
	ssize_t dcount;
	int index,
	  k;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_channel_disconnect_t ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( messip_send_channel_disconnect_t ), strerror(errno) );
		return -1;
	}

#if 0
	TRACE( "channel_disconnect: pid=%d tid=%ld name=%s  sockfd=%d\n",
		  msg.pid, msg.tid, msg.name, sockfd );
#endif

	/*--- Search this channel name ---*/
	LOCK;
	pch = (channel_t **) bsearch( msg.name,
	   channels, nb_channels, sizeof( channel_t * ), bsearch_channels );
	ch = ( pch ) ? *pch : NULL;
	UNLOCK;

	/*--- Reply to the client ---*/
	if ( ch == NULL )
	{
		reply.ok = MESSIP_NOK;
	}
	else
	{
		LOCK;
		for ( index = 0; index < nb_channels; index++ )
		{
			int nb,
			  new_nb;
			channel_t *channel;

			channel = channels[index];
			for ( nb = 0, k = 0; k < channel->nb_clients; k++ )
				if ( channel->cnx_clients[k] == sockfd )
					nb++;
			new_nb = channel->nb_clients - nb;
			if ( new_nb == 0 )
			{
				channel->nb_clients = 0;
				free( channel->cnx_clients );
				channel->cnx_clients = NULL;
			}
			else
			{
				int w,
				 *clients;
				clients = (int *) malloc( sizeof( int ) * new_nb );
				for ( w = 0, k = 0; k < channel->nb_clients; k++ )
				{
					if ( channel->cnx_clients[k] != sockfd )
						clients[w++] = channel->cnx_clients[k];
				}				// for (k)
				assert( w == new_nb );
				channel->nb_clients = new_nb;
				free( channel->cnx_clients );
				channel->cnx_clients = clients;
			}
		}						// for (ch)
		reply.ok = MESSIP_OK;
		UNLOCK;
	}							// else
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( reply ) );

	return MESSIP_OK;

}								// client_channel_disconnect


static void *
thread_client_send_buffered_msg( void *arg )
{
	channel_t			*ch = (channel_t *)arg;
	messip_datasend_t	datasend;
	messip_datareply_t	datareply;
	struct iovec		iovec[3];
	buffered_msg_t		*bmsg;
	int					status;
	ssize_t				dcount;
	uint32_t			len;
	fd_set				ready;
	int					sockfd;
	int					nb, k;
	int					do_reply;
	int					sig;
	sigset_t			set;

	TRACE( "thread_client_send_buffered_msg: pid=%d tid=%ld\n",
		  getpid(  ), pthread_self(  ) );
	for ( ;; )
	{

		/*--- Wait until SIGUSR2 is applied to the thread ---*/
		sigemptyset( &set );
		sigaddset( &set, SIGUSR2 );
		sigwait( &set, &sig );

		LOCK;
		nb = ch->nb_msg_buffered;
		sockfd = ch->bufferedsend_sockfd;
		UNLOCK;

		while ( nb > 0 )
		{

			/*--- Wait until ready to write ---*/
			FD_ZERO( &ready );
			FD_SET( sockfd, &ready );
			status = select( FD_SETSIZE, NULL, &ready, NULL, NULL );

			LOCK;
			if ( ch->nb_msg_buffered == 0 )
			{
				UNLOCK;
				break;
			}
			bmsg = ch->buffered_msg[0];

			/*--- Message to send ---*/
			datasend.flag = MESSIP_FLAG_BUFFERED;
			datasend.pid = bmsg->pid_from;
			datasend.tid = bmsg->tid_from;
			datasend.type = bmsg->type;
			datasend.subtype = bmsg->subtype;
			datasend.datalen = bmsg->datalen;
			UNLOCK;

			/*--- Send a message to the 'server' ---*/
			iovec[0].iov_base = &datasend;
			iovec[0].iov_len  = sizeof( datasend );
			iovec[1].iov_base = &len;
			iovec[1].iov_len  = sizeof( int32_t );
			iovec[2].iov_base = bmsg->data;
			iovec[2].iov_len  = bmsg->datalen;
			dcount = do_writev( sockfd, iovec, 3 );
			if ( dcount != (ssize_t) (sizeof( datasend ) + bmsg->datalen + sizeof( int32_t )) )
			{
				printf( "dcount=%d expected=%d datalen=%d\n",
				   dcount,
				   sizeof( datasend ) + bmsg->datalen + sizeof( int32_t ),
				   bmsg->datalen );
			}
			assert( dcount ==
			   (ssize_t) (sizeof( datasend ) + bmsg->datalen + sizeof( int32_t )) );

			/*--- Now wait for an answer from the server ---*/
			errno = -1;
			iovec[0].iov_base = &datareply;
			iovec[0].iov_len  = sizeof( datareply );
			dcount = do_readv( sockfd, iovec, 1 );

			/*--- Clean-up this message ---*/
			LOCK;
			if ( bmsg->data )
				free( bmsg->data );
			free( bmsg );
			if ( --ch->nb_msg_buffered == 0 )
			{
				free( ch->buffered_msg );
				ch->buffered_msg = NULL;
			}
			else
			{
				for ( k = 1; k <= ch->nb_msg_buffered; k++ )
					ch->buffered_msg[k - 1] = ch->buffered_msg[k];
				ch->buffered_msg = (buffered_msg_t **) realloc( ch->buffered_msg,
				   sizeof( buffered_msg_t * ) * ch->nb_msg_buffered );
			}

			nb = ch->nb_msg_buffered;
			do_reply = ( nb + 1 == ch->maxnb_msg_buffered );
			UNLOCK;

			if ( ( dcount <= 0 ) && ( errno == ECONNRESET ) )
				continue;
			assert( dcount == sizeof( messip_datareply_t ) );

			/*--- Reply to the server, which was blocked because too many buffered messages ---*/
			if ( do_reply )
			{
				messip_reply_buffered_send_t msgreply;

				msgreply.ok = MESSIP_OK;
				msgreply.nb_msg_buffered = nb;
				iovec[0].iov_base = &msgreply;
				iovec[0].iov_len  = sizeof( msgreply );
				dcount = do_writev( ch->reply_on_release_sockfd, iovec, 1 );
				assert( dcount == sizeof( msgreply ) );
			}					// if

		}						// while (nb > 0)

	}							// for (;;)

	/*--- Done ---*/
	pthread_exit( NULL );
	return NULL;

}								// thread_client_send_buffered_msg

static int
client_death_notify( int sockfd,
   struct sockaddr_in *client_addr )
{
	ssize_t dcount;
	channel_t *ch;
	struct iovec iovec[1];
	messip_send_death_notify_t msgsend;
	messip_reply_death_notify_t msgreply;
	connexion_t *cnx;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msgsend;
	iovec[0].iov_len  = sizeof( msgsend );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( msgsend ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( msgsend ), strerror(errno) );
		return -1;
	}

#if 0
	TRACE( "client_death_notify: pid=%d tid=%ld status=%d\n",
		  msgsend.pid_from, msgsend.tid_from, msgsend.status );
#endif

	/*--- Update internal data ---*/
	LOCK;
	ch = search_ch_by_sockfd( sockfd );
	assert( ch != NULL );
	ch->f_notify_deaths = msgsend.status;
	cnx = ch->cnx;
	UNLOCK;

	/*--- Reply to the client ---*/
	msgreply.ok = MESSIP_OK;
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( msgreply ) );

	return 0;

}								// client_death_notify


static int
client_buffered_send( int sockfd,
   struct sockaddr_in *client_addr )
{
	ssize_t dcount;
	channel_t *ch;
	buffered_msg_t *bmsg;
	connexion_t *cnx;
	pthread_attr_t attr;
	void *data;
	struct iovec iovec[1];
	messip_send_buffered_send_t msg;
	messip_reply_buffered_send_t msgreply;
	int nb;
	int do_reply;
	struct sockaddr_in sockaddr;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( msg ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( msg ), strerror(errno) );
		return -1;
	}

	/*--- Read the private message ---*/
	if ( msg.datalen == 0 )
	{
		data = NULL;
	}
	else
	{
		data = malloc( msg.datalen );
		dcount = read( sockfd, data, msg.datalen );
		if ( dcount != msg.datalen )
		{
			fprintf( stderr, "Should have read %d bytes - only %d have been read\n",
			   msg.datalen, dcount );
			return -1;
		}
	}

#if 0
	TRACE( "client_buffered_send: pid=%d tid=%d type=%d subtype=%d %d [%s]\n",
		  msg.pid_from, msg.tid_from, msg.type, msg.subtype, msg.datalen, data );
#endif

	/*--- Create a thread to manage buffered messages ? ---*/
	LOCK;
	ch = search_ch_by_sockfd( msg.mgr_sockfd );
	if ( ch == NULL )
	{
		UNLOCK;
		fprintf( stderr, "%s: socket %d not found\n", __FUNCTION__, msg.mgr_sockfd );
		return -1;
	}
	cnx = ch->cnx;

	/*--- Create socket then connection ---*/
	if ( ch->bufferedsend_sockfd == 0 )
	{
		messip_datasend_t datasend;
		struct iovec iovec[1];
		ssize_t dcount;

		ch->bufferedsend_sockfd = socket( AF_INET, SOCK_STREAM, 0 );
		if ( ch->bufferedsend_sockfd < 0 )
		{
			UNLOCK;
			fprintf( stderr, "%s %d\n\tUnable to open a socket!\n", __FILE__, __LINE__ );
			return -1;
		}

		/*--- Connect socket using name specified ---*/
		memset( &sockaddr, 0, sizeof( sockaddr ) );
		sockaddr.sin_family = AF_INET;
		sockaddr.sin_port = htons( ch->sin_port );
		sockaddr.sin_addr.s_addr = ch->sin_addr;
		if ( connect( ch->bufferedsend_sockfd,
			  ( const struct sockaddr * ) &sockaddr, sizeof( sockaddr ) ) < 0 )
		{
			UNLOCK;
			fprintf( stderr, "%s %d\n\tUnable to connect to host %s, port %d: %s\n",
			   __FILE__, __LINE__,
			   inet_ntoa( sockaddr.sin_addr ), sockaddr.sin_port, strerror(errno) );
			if ( close( ch->bufferedsend_sockfd ) == -1 )
				fprintf( stderr, "Error while closing socket %d: %s\n", ch->bufferedsend_sockfd, strerror(errno) );
			return -1;
		}

		/*--- Send a fake message ---*/
		datasend.flag = MESSIP_FLAG_CONNECTING;
		iovec[0].iov_base = &datasend;
		iovec[0].iov_len  = sizeof( datasend );
		dcount = do_writev( ch->bufferedsend_sockfd, iovec, 1 );
		assert( dcount == sizeof( datasend ) );

	}							// if

	/*--- Update internal queue, managed by the thread client_send_buffered_msg ---*/
	bmsg = (buffered_msg_t *) malloc( sizeof( buffered_msg_t ) );
	bmsg->type = msg.type;
	bmsg->subtype = msg.subtype;
	bmsg->pid_from = msg.pid_from;
	bmsg->tid_from = msg.tid_from;
	bmsg->pid_to = cnx->pid;
	bmsg->tid_to = cnx->tid;
	bmsg->datalen = msg.datalen;
	bmsg->data = data;
	if ( ( nb = ch->nb_msg_buffered ) == 0 )
		ch->buffered_msg = (buffered_msg_t **) malloc( sizeof( buffered_msg_t * ) );
	else
		ch->buffered_msg = (buffered_msg_t **) realloc( ch->buffered_msg,
		   sizeof( buffered_msg_t * ) * ( ch->nb_msg_buffered + 1 ) );
	ch->buffered_msg[ch->nb_msg_buffered++] = bmsg;
	do_reply = ( ch->nb_msg_buffered < ch->maxnb_msg_buffered );
	UNLOCK;

	if ( ch->tid_client_send_buffered_msg == 0 )
	{
		pthread_attr_init( &attr );
		pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
		pthread_create( &ch->tid_client_send_buffered_msg, &attr,
		   thread_client_send_buffered_msg, ch );
	}							// if

	/*--- Signal the buffered msg send thread that there is something ---*/
	pthread_kill( ch->tid_client_send_buffered_msg, SIGUSR2 );

	/*--- Reply to the client ---*/
	if ( do_reply )
	{
		msgreply.ok = MESSIP_OK;
		msgreply.nb_msg_buffered = nb;
		iovec[0].iov_base = &msgreply;
		iovec[0].iov_len  = sizeof( msgreply );
		dcount = do_writev( sockfd, iovec, 1 );
		assert( dcount == sizeof( msgreply ) );
	}							// if
	else
	{
		ch->reply_on_release_sockfd = sockfd;
	}

	return 0;

}								// client_buffered_send


static void *
thread_client_trigger_proxy( void *arg )
{
	proxy_t *proxy = ( proxy_t * ) arg;
	int nb;
	int sockfd;
	fd_set ready;
	uint32_t len;
	ssize_t dcount;
	int status;
	messip_datasend_t datasend;
	messip_datareply_t datareply;
	struct iovec iovec[3];
	int sig;
	sigset_t set;

//	logg( NULL, "thread_client_trigger_proxy: pid=%d tid=%ld\n",
//		  getpid(  ), pthread_self(  ) );
	for ( ;; )
	{

		sigemptyset( &set );
		sigaddset( &set, SIGUSR2 );
		sigwait( &set, &sig );

		LOCK;
		nb = proxy->nb_proxies_to_trigger;
		sockfd = proxy->process_to_trigger_sockfd;
		UNLOCK;

		while ( nb > 0 )
		{
			int tlen;

			/*--- Wait until ready to write ---*/
			FD_ZERO( &ready );
			FD_SET( sockfd, &ready );
			status = select( sockfd+1, NULL, &ready, NULL, NULL );
			if ( status <= 0 )
				printf( "OOOPSS!!! status=%d: %s\n", status, strerror(errno) );
			assert( status > 0 );
			if ( !FD_ISSET( sockfd, &ready ) )
			{
				printf( "YYOOOPSS!!! fd=%d \n", sockfd );
			}

			LOCK;
			nb = proxy->nb_proxies_to_trigger;
			if ( nb == 0 )
			{
				UNLOCK;
				break;
			}

			/*--- Message to send ---*/
			datasend.flag = MESSIP_FLAG_PROXY;
			datasend.pid = -1;	// Unknown
			datasend.tid = -1;	// Unknown
			datasend.type = proxy->proxy_index;
			datasend.subtype = 10000+(int) (9999.0*rand()/(RAND_MAX+1.0));
			datasend.datalen = proxy->nbytes;
			UNLOCK;

			/*--- Send a message to the 'server' ---*/
			iovec[0].iov_base = &datasend;
			iovec[0].iov_len  = sizeof( datasend );
			iovec[1].iov_base = &len;
			iovec[1].iov_len  = sizeof( int32_t );
//			logg( NULL, "signal proxy %d  data=%p bytes=%d\n", proxy->proxy_index, proxy->data, proxy->nbytes );
			iovec[2].iov_base = proxy->data;
			iovec[2].iov_len  = proxy->nbytes;
			dcount = do_writev( sockfd, iovec, 3 );
			tlen = sizeof( datasend ) + sizeof( int32_t ) + proxy->nbytes;
			if ( dcount != tlen )
				printf( "dcount=%d expected=%d nbytes=%d - %d - data=%p: %s\n",
				   dcount, tlen, proxy->nbytes,
				   sizeof( datasend ), proxy->data, strerror(errno) );
			assert( dcount == tlen );

			/*--- Now wait for an answer from the server ---*/
			errno = -1;
			iovec[0].iov_base = &datareply;
			iovec[0].iov_len  = sizeof( datareply );
			dcount = do_readv( sockfd, iovec, 1 );

			/*--- Clean-up this message ---*/
			LOCK;
			nb = --proxy->nb_proxies_to_trigger;
			UNLOCK;

			if ( ( dcount <= 0 ) && ( errno == ECONNRESET ) )
				continue;
			if ( dcount == 0 )
				continue;
#if 0
			if ( dcount != sizeof( messip_datareply_t ) )
				fprintf( stderr, "%s %d: dcount=%d: %s\n",
				   __FILE__, __LINE__, dcount, strerror(errno) );
#endif
//          assert( dcount == sizeof(messip_datareply_t) );

		}						// while (nb > 0)

	}							// for (;;)

	/*--- Done ---*/
	pthread_exit( NULL );
	return NULL;

}								// thread_client_trigger_proxy


static int
client_proxy_attach( int sockfd,
   struct sockaddr_in *client_addr )
{
	ssize_t dcount;
	channel_t *ch;
	connexion_t *cnx;
	void *data;
	struct iovec iovec[1];
	messip_send_proxy_attach_t msgsent;
	messip_reply_proxy_attach_t msgreply;
	struct sockaddr_in sockaddr;
	proxy_t *proxy;
	int proxy_index;
	pthread_attr_t attr;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msgsent;
	iovec[0].iov_len  = sizeof( msgsent );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d\n\t%s\n",
			__FILE__, __LINE__,
			strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_proxy_attach_t ) )
	{
		fprintf( stderr, "%s %d\n\tread %d of %d: %s\n",
		   __FILE__, __LINE__,
		   dcount, sizeof( messip_send_proxy_attach_t ), strerror(errno) );
		return -1;
	}

	/*--- Read the private message ---*/
	if ( msgsent.nbytes == 0 )
	{
		data = NULL;
	}
	else
	{
		data = malloc( msgsent.nbytes );
		dcount = read( sockfd, data, msgsent.nbytes );
		if ( dcount != msgsent.nbytes )
		{
			fprintf( stderr, "%s %d\n\tShould have read %d bytes - only %d have been read\n",
			   __FILE__, __LINE__,
			   msgsent.nbytes, dcount );
			return -1;
		}
	}

#if 0
	TRACE( "*** client_proxy_attach: from: pid=%d,tid=%ld to: pid=%d nbytes=%d [data=%p] sockfd=%d\n",
		  msgsent.pid_from, msgsent.tid_from,
		  msgsent.pid_to,
		  msgsent.nbytes, data, sockfd );
#endif

	/*--- Create a thread to manage buffered messages ? ---*/
	LOCK;
	ch = search_ch_by_pid( msgsent.pid_to );
	if ( ch == NULL )
	{
		UNLOCK;
		msgreply.ok = MESSIP_NOK;
		iovec[0].iov_base = &msgreply;
		iovec[0].iov_len  = sizeof( msgreply );
		dcount = do_writev( sockfd, iovec, 1 );
		assert( dcount == sizeof( msgreply ) );
		return 0;
	}
	cnx = ch->cnx;

	/*--- Update the internal table of proxies ---*/
	proxy = (proxy_t *) malloc( sizeof( proxy_t ) );
	if ( proxies_sz == 0 )
	{
		proxies = (proxy_t **) malloc( sizeof( proxy_t * ) );
		proxy_index = 0;
		nb_proxies = 1;
		proxies_sz = 1;
	}
	else
	{
		for ( proxy_index = 0; proxy_index < proxies_sz; proxy_index++ )
			if ( proxies[proxy_index]->nbytes == -1 )
				break;
		if ( proxy_index == proxies_sz )
		{
			proxies = (proxy_t **) realloc( proxies, sizeof( proxy_t * ) * ( proxies_sz + 1 ) );
			proxy_index = proxies_sz++;
		}
		nb_proxies++;
	}
	proxies[proxy_index] = proxy;
	proxy->pid_from = msgsent.pid_from;
	proxy->tid_from = msgsent.tid_from;
	proxy->pid_to = msgsent.pid_to;
	proxy->proxy_index = proxy_index;
	proxy->data = data;
	proxy->nbytes = msgsent.nbytes;
	proxy->priority = msgsent.priority;
	proxy->process_to_trigger_sockfd = -1;
	proxy->nb_proxies_to_trigger = 0;

	/*--- Create thread to Trigger this proxy ---*/
	pthread_attr_init( &attr );
	pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
	pthread_create( &proxy->tid_client_proxies_to_trigger, &attr,
	   thread_client_trigger_proxy, proxy );
	UNLOCK;

	/*--- Create socket then connection ---*/
	if ( proxy->process_to_trigger_sockfd == -1 )
	{
		messip_datasend_t datasend;
		struct iovec iovec[1];
		ssize_t dcount;

		proxy->process_to_trigger_sockfd = socket( AF_INET, SOCK_STREAM, 0 );
		if ( proxy->process_to_trigger_sockfd < 0 )
		{
			UNLOCK;
			fprintf( stderr, "%s %d\n\tUnable to open a socket!\n",
				__FILE__, __LINE__ );
			return -1;
		}

		/*--- Connect socket using name specified ---*/
		memset( &sockaddr, 0, sizeof( sockaddr ) );
		sockaddr.sin_family = AF_INET;
		sockaddr.sin_port = htons( ch->sin_port );
		sockaddr.sin_addr.s_addr = ch->sin_addr;
		if ( connect( proxy->process_to_trigger_sockfd,
			  ( const struct sockaddr * ) &sockaddr, sizeof( sockaddr ) ) < 0 )
		{
			UNLOCK;
			fprintf( stderr, "%s %d\n\tUnable to connect to host %s, port %d: %s\n",
			   __FILE__, __LINE__,
			   inet_ntoa( sockaddr.sin_addr ), sockaddr.sin_port, strerror(errno) );
			if ( close( proxy->process_to_trigger_sockfd ) == -1 )
				fprintf( stderr, "Error while closing socket %d: %s\n",
						proxy->process_to_trigger_sockfd, strerror(errno) );
			return -1;
		}

		/*--- Send a fake message ---*/
		datasend.flag = MESSIP_FLAG_CONNECTING;
		iovec[0].iov_base = &datasend;
		iovec[0].iov_len  = sizeof( datasend );
		dcount = do_writev( proxy->process_to_trigger_sockfd, iovec, 1 );
		assert( dcount == sizeof( datasend ) );

	}							// if

	/*--- Reply to the client ---*/
	msgreply.ok = MESSIP_OK;
	msgreply.proxy = proxy_index;
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( msgreply ) );

	return 0;

}								// client_proxy_attach


static int
client_proxy_detach( int sockfd,
   struct sockaddr_in *client_addr )
{
	ssize_t dcount;
	struct iovec iovec[1];
	messip_send_proxy_detach_t msgsent;
	messip_reply_proxy_detach_t msgreply;
	proxy_t *proxy;
	int index,
	   f_err;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msgsent;
	iovec[0].iov_len  = sizeof( msgsent );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d\n\t%s\n",
			__FILE__, __LINE__,
			strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_proxy_detach_t ) )
	{
		fprintf( stderr, "%s %d\n\tread %d of %d: %s\n",
		   __FILE__, __LINE__,
		   dcount, sizeof( messip_send_proxy_detach_t ), strerror(errno) );
		return -1;
	}

	/*--- Destroy this proxy ---*/
	index = msgsent.proxy;
	f_err = 0;
	if ( ( index < 0 ) || ( index >= proxies_sz ) )
		f_err = 1;
	if ( !f_err )
	{
		proxy = proxies[index];
		if ( proxy->tid_client_proxies_to_trigger == 0 )
		{
			f_err = 1;
		}
		else
		{
			pthread_cancel( proxy->tid_client_proxies_to_trigger );
			if ( close( proxy->process_to_trigger_sockfd ) == -1 )
				fprintf( stderr, "Error while closing socket %d: %s\n",
					proxy->process_to_trigger_sockfd, strerror(errno) );
			proxy->tid_client_proxies_to_trigger = 0;
			proxy->process_to_trigger_sockfd = -1;
		}						// if
		proxy->nbytes = -1;		// Slot is available now
		nb_proxies--;			// Nb of proxies which are alive
	}							// for (proxy)

	/*--- Reply to the client ---*/
	msgreply.ok = (f_err) ? MESSIP_NOK : MESSIP_OK;
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( msgreply ) );

	return 0;

}								// client_proxy_detach


static int
client_proxy_trigger( int sockfd,
   struct sockaddr_in *client_addr )
{
	ssize_t dcount;
	proxy_t *proxy = NULL;
	struct iovec iovec[1];
	messip_send_proxy_trigger_t msgsent;
	messip_reply_proxy_trigger_t msgreply;
	int index,
	  f_err;

	/*
		Read additional data specific to this message
	*/
	iovec[0].iov_base = &msgsent;
	iovec[0].iov_len  = sizeof( msgsent );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_proxy_trigger_t ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( messip_send_proxy_trigger_t ), strerror(errno) );
		return -1;
	}

#if 0
	TRACE( "client_proxy_trigger: from: pid=%d,tid=%ld to: pid=%d \n",
		  msgsent.pid_from, msgsent.tid_from, msgsent.pid_to );
#endif

	/*
		Update internal queue, managed by the thread client_trigger_proxy
	*/
	index = msgsent.pid_to;
	f_err = 0;
	if ( ( index < 0 ) || ( index >= proxies_sz ) )
		f_err = 1;
	LOCK;
	if ( !f_err )
	{
		proxy = proxies[index];
		if ( proxy->nbytes == -1 )
			f_err = 1;
		else
			proxy->nb_proxies_to_trigger++;
	}

	/*
		Signal the proxies_to_trigger thread that there is something
	*/
	if ( !f_err )
	{
//		TRACE( "Send SIGUSR1 to tid=%ld proxy=%d\n", proxy->tid_client_proxies_to_trigger, proxy->proxy_index );
		pthread_kill( proxy->tid_client_proxies_to_trigger, SIGUSR2 );
	}
	UNLOCK;

	/*
		Reply to the client
	*/
	if ( !f_err )
	{
		msgreply.ok = MESSIP_OK;
		msgreply.pid_owner = proxy->pid_to;
	}
	else
	{
		msgreply.ok = MESSIP_NOK;
	}
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( msgreply ) );

	return 0;

}								// client_proxy_trigger


static int
client_proxy_get_owner( int sockfd,
   struct sockaddr_in *client_addr )
{
	ssize_t dcount;
	proxy_t *proxy = NULL;
	struct iovec iovec[1];
	messip_send_proxy_get_owner_t msgsent;
	messip_reply_proxy_get_owner_t msgreply;
	int index;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msgsent;
	iovec[0].iov_len  = sizeof( msgsent );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_proxy_get_owner_t ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( messip_send_proxy_get_owner_t ), strerror(errno) );
		return -1;
	}

#if 1
	TRACE( "client_proxy_get_owner: from: pid=%d,tid=%ld to: pid=%d \n",
		  msgsent.pid_from, msgsent.tid_from, msgsent.pid_to );
#endif

	/*
		Query internal queue, managed by the thread client_trigger_proxy
	*/
	msgreply.ok = MESSIP_NOK;
	LOCK;
	for ( index = 0; index < proxies_sz; index++ )
	{
		proxy = proxies[index];
		if ( proxy->nbytes == -1 )	// Slot available ?
			continue;
		if ( index == msgsent.pid_to )
		{
			msgreply.ok = MESSIP_OK;
			msgreply.pid_owner = proxy->pid_to;
			break;
		}						// if
	}							// for
	UNLOCK;

	/*
		Reply to the client
	*/
	iovec[0].iov_base = &msgreply;
	iovec[0].iov_len  = sizeof( msgreply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( msgreply ) );

	return 0;

}								// client_proxy_get_owner


#ifdef MESSIP_INFORM_STATE

static int
client_debug_op_inform_state( int sockfd,
   struct sockaddr_in *client_addr )
{
	connexion_t *cnx;
	struct iovec iovec[1];
	messip_send_inform_messipmgr_t msg;
	messip_reply_inform_messipmgr_t reply;
	ssize_t dcount;

	/*--- Read additional data specific to this message ---*/
	iovec[0].iov_base = &msg;
	iovec[0].iov_len  = sizeof( msg );
	dcount = do_readv( sockfd, iovec, 1 );
	if ( dcount == -1 )
	{
		fprintf( stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno) );
		return -1;
	}
	if ( dcount != sizeof( messip_send_inform_messipmgr_t ) )
	{
		fprintf( stderr, "%s %d: read %d of %d: %s\n",
		   __FILE__, __LINE__, dcount, sizeof( messip_send_inform_messipmgr_t ), strerror(errno) );
		return -1;
	}

#if 1
	TRACE( "client_debug_op_inform_state: status=%d  pid=%d,%d tid=%ld,%ld  sockfd=%d\n",
		  msg.status, msg.pid, msg.pid_blocked_on, msg.tid, msg.tid_blocked_on, sockfd );
#endif

	/*--- Search this connexion and update it---*/
	LOCK;
	cnx = search_cnx_by_sockfd( sockfd );
	if ( cnx != NULL )
	{
		cnx->state = msg.status;
		cnx->pid_blocked_on  = msg.pid_blocked_on;
		cnx->tid_blocked_on  = msg.tid_blocked_on;
		cnx->when_blocked_on = msg.when_blocked_on;
	}
	UNLOCK;

	/*--- Reply to the client ---*/
	reply.ok = MESSIP_OK;
	iovec[0].iov_base = &reply;
	iovec[0].iov_len  = sizeof( reply );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( reply ) );

	return MESSIP_OK;

}								// client_debug_op_inform_state

#endif


/*
	return 1 if socket must exist in table, else 0
*/

static int
handle_client_msg( int sockfd,
   struct sockaddr_in *client_addr,
   int32_t op,
   connexion_t ** new_cnx )
{

	switch ( op )
	{

		case MESSIP_OP_CONNECT:
			handle_client_connect( sockfd, client_addr, new_cnx );
			return 1;

		case MESSIP_OP_CHANNEL_CREATE:
			client_channel_create( sockfd, client_addr, new_cnx );
			return 1;

		case MESSIP_OP_CHANNEL_DELETE:
			client_channel_delete( sockfd, client_addr );
			return 1;

		case MESSIP_OP_CHANNEL_CONNECT:
			client_channel_connect( sockfd, client_addr );
			return 1;

		case MESSIP_OP_CHANNEL_DISCONNECT:
			client_channel_disconnect( sockfd, client_addr );
			return 1;

		case MESSIP_OP_BUFFERED_SEND:
			client_buffered_send( sockfd, client_addr );
			return 1;

		case MESSIP_QNX_OP_PROXY_ATTACH:
			client_proxy_attach( sockfd, client_addr );
			return 1;

		case MESSIP_QNX_OP_PROXY_DETACH:
			client_proxy_detach( sockfd, client_addr );
			return 1;

		case MESSIP_QNX_OP_PROXY_TRIGGER:
			client_proxy_trigger( sockfd, client_addr );
			return 1;

		case MESSIP_QNX_OP_PROXY_GET_OWNER:
			client_proxy_get_owner( sockfd, client_addr );
			return 1;

		case MESSIP_OP_DEATH_NOTIFY :
			client_death_notify( sockfd, client_addr );
			return 1;

		case MESSIP_OP_SIN :
			LOCK;
			debug_show(  );
			UNLOCK;
			return 0;

#ifdef MESSIP_INFORM_STATE
		case MESSIP_DEBUG_OP_INFORM_STATE :
			client_debug_op_inform_state( sockfd, client_addr );
			break;
#endif

		default:
			fprintf( stderr, "%s %d:\n\tUnknown code op %d - 0x%08X\n",
			   __FILE__, __LINE__, op, op );
			break;

	}							// switch (op)

	return 0;

}								// handle_client_msg


static int
notify_server_death_client( channel_t * ch,
   pid_t pid,
   pthread_t tid,
   int code )
{
	messip_datasend_t datasend;
	int sockfd;
	struct iovec iovec[1];
	int status;
	ssize_t dcount;
	fd_set ready;
	struct sockaddr_in sockaddr;

	sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if ( sockfd < 0 )
	{
		fprintf( stderr, "%s %d\n\tUnable to open a socket!\n", __FILE__, __LINE__ );
		return -1;
	}

	/*--- Connect socket using name specified ---*/
	memset( &sockaddr, 0, sizeof( sockaddr ) );
	sockaddr.sin_family = AF_INET;
	sockaddr.sin_port = htons( ch->sin_port );
	sockaddr.sin_addr.s_addr = ch->sin_addr;
	if ( connect( sockfd, ( const struct sockaddr * ) &sockaddr, sizeof( sockaddr ) ) < 0 )
	{
		if ( errno != ECONNREFUSED )
		    perror("connect()");
			fprintf( stderr, "%s %d\n\tUnable to connect to host %s, port %d: %s\n",
			   __FILE__, __LINE__,
			   inet_ntoa( sockaddr.sin_addr ), sockaddr.sin_port, strerror(errno) );
		if ( close( sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n",
				sockfd, strerror(errno) );
		return -1;
	}

	/*--- Wait until enable to write ---*/
	FD_ZERO( &ready );
	FD_SET( sockfd, &ready );
	status = select( FD_SETSIZE, NULL, &ready, NULL, NULL );

	/*--- Send a fake message ---*/
	memset( &datasend, 0, sizeof( messip_datasend_t ) );
	datasend.flag = MESSIP_FLAG_CONNECTING;
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	dcount = do_writev( sockfd, iovec, 1 );
	assert( dcount == sizeof( datasend ) );

	/*--- Message to send ---*/
	datasend.flag    = code;
	datasend.pid     = pid;
	datasend.tid     = tid;
	datasend.type    = -1;
	datasend.subtype = -1;
	datasend.datalen = 0;

	/*--- Send a message to the 'server' ---*/
	iovec[0].iov_base = &datasend;
	iovec[0].iov_len  = sizeof( datasend );
	dcount = do_writev( sockfd, iovec, 1 );
//	assert( dcount == sizeof( datasend ) );

	/*--- ok ---*/
	if ( close( sockfd ) == -1 )
		fprintf( stderr, "Error while closing socket %d: %s\n",
			sockfd, strerror(errno) );
	return 0;

}								// notify_server_death_client


static void *
thread_client_thread( void *arg )
{
	clientdescr_t *descr = ( clientdescr_t * ) arg;
	ssize_t dcount;
	int32_t op;
	int index,
	  found;
	connexion_t *new_cnx,
	**cnx,
	 *connexion;
	channel_t **ch,
	 *channel;
	int k;
	struct iovec iovec[1];
	pid_t pid;
	pthread_t tid;
	proxy_t *proxy;
	int search_socket=0;

#if 0
	TRACE( "thread_client_thread: pid=%d tid=%ld\n", getpid(  ), pthread_self(  ) );
#endif

	for ( new_cnx = NULL;; )
	{

		iovec[0].iov_base = &op;
		iovec[0].iov_len  = sizeof( int32_t );
		dcount = do_readv( descr->sockfd_accept, iovec, 1 );
		if ( dcount <= 0 )
			break;

		if ( dcount != sizeof( int32_t ) )
		{
			fprintf( stderr, "%s %d:\n\tread %d byte[%08X], should have read %d bytes\n",
			   __FILE__, __LINE__, dcount, op, sizeof( int32_t ) );
			break;
		}

		// Operation code is always sent as big endian (JAVA do so!)
#if BYTE_ORDER == LITTLE_ENDIAN
		op = int_little_endian( op );
#endif

		search_socket = handle_client_msg( descr->sockfd_accept,
			&descr->client_addr, op, &new_cnx );

	}							// for (;;)

	/*--- Close the connection ---*/
	shutdown( descr->sockfd_accept, SHUT_RDWR );

	if ( !search_socket )
	{
		if ( close ( descr->sockfd_accept ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n",
				descr->sockfd_accept, strerror(errno) );
		free( descr );
		pthread_exit( NULL );
		return NULL;
	}

	/*--- Destroy this connection ---*/
	LOCK;
	for ( cnx = connexions, found = 0, index = 0;
		  index < nb_connexions;
		  index++, cnx++ )
	{
		if ( ( *cnx )->sockfd == descr->sockfd_accept )
		{
			found = 1;
			break;
		}
	}
	if ( !found )
	{
		fprintf( stderr, "%s %d:\n\tfound should be true\n", __FILE__, __LINE__ );
		UNLOCK;
		if ( close ( descr->sockfd_accept ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n",
				descr->sockfd_accept, strerror(errno) );
		free( descr );
		pthread_exit( NULL );
		return NULL;
	}
	connexion = connexions[index];

	logg( LOG_MESSIP_INFORMATIVE, "Destroy connexion #%d sockfd=%-3d pid=%d [%s]\n",
		  index, connexion->sockfd, connexion->pid, connexion->process_name );
	if ( close( connexion->sockfd ) == -1 )
		fprintf( stderr, "Unable to close socket %d: %s\n",
			connexion->sockfd, strerror(errno) );
	pid = connexion->pid;
	tid = connexion->tid;
	for ( k = index + 1; k < nb_connexions; k++ )
		connexions[k - 1] = connexions[k];
	if ( connexion->nb_cnx_channels )
		free( connexion->sockfd_cnx_channels );
	free( connexion );
	if ( --nb_connexions == 0 )
		connexions = NULL;

	/*--- Notify all owners of connected channels that this client dismissed ---*/
	for ( index = 0; index < nb_channels; index++ )
	{
		int nb,
		  new_nb;

		channel = channels[index];
		for ( nb = 0, k = 0; k < channel->nb_clients; k++ )
		{
			if ( channel->cnx_clients[k] == descr->sockfd_accept )
			{
				notify_server_death_client( channel, pid, tid, MESSIP_FLAG_DISMISSED );
				nb++;
			}					// if
		}						// for (k)
		new_nb = channel->nb_clients - nb;
		if ( new_nb == 0 )
		{
			channel->nb_clients = 0;
			free( channel->cnx_clients );
			channel->cnx_clients = NULL;
		}
		else
		{
			int w,
			 *clients;
			clients = (int *) malloc( sizeof( int ) * new_nb );
			for ( w = 0, k = 0; k < channel->nb_clients; k++ )
			{
				if ( channel->cnx_clients[k] != descr->sockfd_accept )
					clients[w++] = channel->cnx_clients[k];
			}					// for (k)
			assert( w == new_nb );
			channel->nb_clients = new_nb;
			free( channel->cnx_clients );
			channel->cnx_clients = clients;
		}
	}							// for (index)

	/*--- Notify other processes (optional) that this process is now dead---*/
	for ( ch = channels, index = 0; index < nb_channels; index++, ch++ )
	{
		if ( ( *ch )->cnx == connexion )
			continue;

		channel = channels[index];
		if ( !channel->f_notify_deaths )
			continue;

		notify_server_death_client( channel, pid, tid, MESSIP_FLAG_DEATH_PROCESS );

	}							// for (index)

	/*--- Destroy all channels related to this connection, if any ---*/
	for ( ch = channels, index = 0; index < nb_channels; index++, ch++ )
	{
		if ( ( *ch )->cnx == connexion )
		{
			channel = channels[index];
			destroy_channel( channel, index );
			index--; ch--;
		}						// if
	}							// for (ch)
	UNLOCK;

	/*--- Any proxy owned by this client and to been then destroyed ? ---*/
	for ( index = 0; index < proxies_sz; index++ )
	{
		proxy = proxies[index];
		if ( proxy->nbytes == -1 )
			continue;
		if ( proxy->pid_to != pid )
			continue;
		if ( proxy->tid_client_proxies_to_trigger )
		{
			pthread_cancel( proxy->tid_client_proxies_to_trigger );
			if ( close( proxy->process_to_trigger_sockfd ) == -1 )
				fprintf( stderr, "Error while closing socket %d: %s\n",
					proxy->process_to_trigger_sockfd, strerror(errno) );
			proxy->tid_client_proxies_to_trigger = 0;
			proxy->process_to_trigger_sockfd = -1;
		}						// if
		proxy->nbytes = -1;		// Slot is available now
		nb_proxies--;			// Nb of proxies which are alive
	}							// for (proxy)

	/*--- Done ---*/
	free( descr );
	pthread_exit( NULL );
	return NULL;
}								// thread_client_thread


static void
sigint_sighandler( int signum )
{
	connexion_t *cnx;
	channel_t *ch;
	int index;

	LOCK;

	for ( index = 0; index < nb_channels; index++ )
	{
		ch = channels[index];
		destroy_channel( ch, index );
	}							// for

	for ( index = 0; index < nb_connexions; index++ )
	{
		cnx = connexions[index];
		if ( close( cnx->sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n",
				cnx->sockfd, strerror(errno) );
	}							// for

	UNLOCK;
	f_bye = 1;
	exit(1);

}								// sigint_sighandler


int
main( int argc,
   char *argv[] )
{
	int sockfd;
	struct sockaddr_in server_addr;
	int status;
	int reuse;
	pthread_t tid;
	pthread_attr_t attr;
	char hostname[64] = "localhost";
	struct sigaction sa;
	sigset_t set;
	int port, port_http;
	int c;
	/*
	int option_index, c;
	static struct option long_options[] = {
		{ "port", 1, NULL, 'p' },
		{ "log", 1, NULL, 'l' },
		{ NULL, 0, NULL, 0 }
	};
	*/

	/*
	printf("sizeof(pthread_t) = %d\n", sizeof(pthread_t));
	*/

	fprintf( stdout, "To stop It:    kill -s SIGINT  %d\n", getpid(  ) );
	f_bye = 0;					// Set to 1 when SIGINT has been applied

	/*--- Read /erx/messip if exist ---*/
	port = MESSIP_DEFAULT_PORT;
	port_http = port + 1;
	if ( access( "/etc/messip", F_OK ) == 0 )
		read_etc_messip( hostname, &port, &port_http );

	/*--- Any parameter ? ---*/
	logg_dir = NULL;
	for (;;)
	{
		//c = getopt_long( argc, argv, "p:l:", long_options, &option_index );
		c = getopt( argc, argv, "p:l:");
		if ( c == -1 )
			break;
//		printf( "c=%d option_index=%d arg=[%s]\n", c, option_index, optarg );
		switch ( c )
		{
			case 'p' :
				port = atoi( optarg );
				break;
			case 'l' :
				logg_dir = optarg;
				break;
			case 'h' :
				port_http = atoi( optarg );
				break;
		}						// switch
	}							// for (;;)

	fprintf( stdout, "Using %s:%d for Messaging\n",
		hostname, port );
	fprintf( stdout, "Using %s:%d for http\n",
		hostname, port_http );

	/*--- Initialize active connexions and channels ---*/
	nb_connexions = 0;
	connexions = NULL;
	nb_channels = 0;
	channels = NULL;

	/*--- Register a function that clean-up opened sockets when exiting ---*/
	sa.sa_handler = sigint_sighandler;
	sa.sa_flags = 0;
	sigemptyset( &sa.sa_mask );
	sigaction( SIGINT, &sa, NULL );

	sigemptyset( &set );
	sigaddset( &set, SIGUSR2 );
	pthread_sigmask( SIG_BLOCK, &set, NULL );

	/*--- Create a mutex, in order to protect shared table of data ---*/
	if ( pthread_mutex_init( &mutex, NULL ) == -1 )
	{
		fprintf( stderr, "%s %d\n\tUnable to initialize mutex: %s\n",
		   __FILE__, __LINE__, strerror(errno) );
		return -1;
	}

	/*--- Create socket ---*/
	sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if ( sockfd < 0 )
	{
		fprintf( stderr, "%s %d\n\tUnable to open a socket!\n", __FILE__, __LINE__ );
		return -1;
	}

	reuse = 1;

	if ((setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
					(void *) &reuse, sizeof(int))) < 0) {
		fprintf( stderr, "%s %d\n\tUnable to set SO_REUSEADDR a socket!\n", __FILE__, __LINE__ );
		exit( -1 );
	}

	/*--- Bind the socket ---*/
	memset( &server_addr, 0, sizeof( server_addr ) );
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl( INADDR_ANY );
	server_addr.sin_port = htons( port );
	status =
	   bind( sockfd, ( struct sockaddr * ) &server_addr, sizeof( struct sockaddr_in ) );
	if ( status == EADDRINUSE )
	{
		fprintf( stderr, "%s %d\n\tUnable to bind, port %d is already in use\n",
		   __FILE__, __LINE__,
		   port );
		if ( close( sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n",
				sockfd, strerror(errno) );
		return -1;
	}
	if ( status < 0 )
	{
		fprintf( stderr, "%s %d\nUnable to bind - port %d - status=%d: %s\n",
			__FILE__, __LINE__,
			port,
			status, strerror( errno ) );
		if ( close( sockfd ) == -1 )
			fprintf( stderr, "Error while closing socket %d: %s\n",
				sockfd, strerror(errno) );
		return -1;
	}

	listen( sockfd, 8 );

	/*--- Create a specific thread to debug information (apply SIGUSR1) ---*/
	pthread_attr_init( &attr );
	pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
	pthread_create( &tid, &attr, &debug_thread, NULL );

	/*--- Create a specific thread to debug information (HTTP server) ---*/
	pthread_attr_init( &attr );
	pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
	pthread_create( &tid, &attr, &http_thread, &port_http );

	for ( ; !f_bye; )
	{
		clientdescr_t *descr;
		pthread_t tid;
		pthread_attr_t attr;

		descr = (clientdescr_t *) malloc( sizeof( clientdescr_t ) );
		descr->client_addr_len = sizeof( struct sockaddr_in );
		descr->sockfd_accept = accept( sockfd,
		   ( struct sockaddr * ) &descr->client_addr, &descr->client_addr_len );
		if ( descr->sockfd_accept == -1 )
		{
			if ( errno == EINTR )	// A signal has been applied
				continue;
			fprintf( stderr, "Socket non accepted: %s\n", strerror(errno) );
			if ( close( sockfd ) == -1 )
				fprintf( stderr, "Error while closing socket %d: %s\n",
					sockfd, strerror(errno) );
			return -1;
		}
#if 0
		logg( LOG_MESSIP_DEBUG_LEVEL1, "accepted a msg from %s, port=%d, socket=%d\n",
			  inet_ntoa( descr->client_addr.sin_addr ),
			  descr->client_addr.sin_port, descr->sockfd_accept );
#endif
		pthread_attr_init( &attr );
		pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
		pthread_create( &tid, &attr, &thread_client_thread, descr );
	}							// for (;;)

	if ( close( sockfd ) == -1 )
		fprintf( stderr, "Error while closing socket %d: %s\n",
			sockfd, strerror(errno) );
	return 0;

}								// main
