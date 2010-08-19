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
#include <time.h>
#include <limits.h>
#include <pthread.h>
#ifdef __FreeBSD__
#include <sys/uio.h>
#endif

#include "messip.h"
#include "messip_private.h"

#include "messip_utils.h"

#include "logg_messip.h"


char *logg_dir;				// Specified by --l or set to NULL

// Updated by messip_logg_on() and messip_logg_off()
static	int	is_logg = 1;

/*
	In case the logg_mgr would be not be there!!
*/
static int
file_logg(
	FILE *output,
	logg_type_t type,
	char *text )
{
	static int seqnb=0;
	struct timespec ts;

	// get the time as soon as possible
	if(clock_gettime(CLOCK_REALTIME, &ts) == -1) {
		perror("clock_gettime()");
		return -1;
	}

	if ( logg_dir )
	{
		char filename[PATH_MAX];
		time_t now;
		struct tm *tm;
		FILE * f;

		time( &now );
		tm = localtime( &now );
		sprintf( filename, "%s/%04d-%02d-%02d",
			logg_dir,
			tm->tm_year + 1900,
			tm->tm_mon + 1,
			tm->tm_mday );
		f = fopen( filename, "a" );
		if ( !f ) {
			perror("fopen()");
			return -1;
		}

		/*
			Write the log data into the locked file
		*/
		if ( text )
		{
			const char *stype;

			switch ( type )
			{
				case LOG_MESSIP_DEBUG_LEVEL1 	: stype = "debug-level1"; break;
				case LOG_MESSIP_DEBUG_LEVEL2  	: stype = "debug-level2"; break;
				case LOG_MESSIP_DEBUG_LEVEL3 	: stype = "debug-level3"; break;
				case LOG_MESSIP_INFORMATIVE	 	: stype = "info"; break;
				case LOG_MESSIP_WARNING		 	: stype = "warning"; break;
				case LOG_MESSIP_NON_FATAL_ERROR : stype = "non-fatal-error"; break;
				case LOG_MESSIP_FATAL_ERROR	 	: stype = "fatal-error"; break;
				case LOG_MESSIP_NOT_YET_DONE	: stype = "not-yet-done"; break;
				default : stype="?"; break;
			}						// switch (type)
			if(fprintf( f, "%6d %9lu.%09lu %-15s %6d %-15s: %s",
				seqnb++,
				ts.tv_sec, ts.tv_nsec,
				stype,
				getpid(),
				"messip_mgr",
				text) < 0) {
				perror("fprintf()");
			}
		}							// if

		fclose( f );
	}							// if ( logg_dir )

	/*
		Display also on the console ?
	*/
	if ( output )
		fputs( text, output );

	return seqnb;

}								// file_logg

pthread_mutex_t logfile_mutex = PTHREAD_MUTEX_INITIALIZER;

int
logg(
	logg_type_t type,
	const char *fmt,
	... )
{
	int saved_errno = errno;
	int seqnb;

	if ( !is_logg )
		return 0;

	/*
		Write the log data into the locked file
	*/
	if ( fmt != NULL )
	{
		va_list ap;
		FILE *output=stderr;
		char text[400+1];
		va_start( ap, fmt );
		vsnprintf( text, 400, fmt, ap );
		va_end( ap );
		if ( logg_dir )
		{
			switch ( type )
			{
				case LOG_MESSIP_DEBUG_LEVEL1 	:
				case LOG_MESSIP_DEBUG_LEVEL2 	:
				case LOG_MESSIP_DEBUG_LEVEL3 	:
				case LOG_MESSIP_INFORMATIVE	 	: output = NULL; break;
				case LOG_MESSIP_WARNING		 	: output = stdout; break;
				case LOG_MESSIP_NON_FATAL_ERROR :
				case LOG_MESSIP_FATAL_ERROR	 	: output = stderr; break;
				case LOG_MESSIP_NOT_YET_DONE	: output = stdout; break;
			}					// switch (type)
		}						// if
		if(pthread_mutex_lock(&logfile_mutex)) {
			fprintf(stderr, "locking logfile_mutex failed\n");
		}

		seqnb = file_logg( output, type, text );

		if(pthread_mutex_unlock(&logfile_mutex)) {
			fprintf(stderr, "unlocking logfile_mutex failed\n");
		}
	}							// if
	else
	{
		if(pthread_mutex_lock(&logfile_mutex)) {
			fprintf(stderr, "locking logfile_mutex failed\n");
		}

		seqnb = file_logg( NULL, type, NULL );

		if(pthread_mutex_unlock(&logfile_mutex)) {
			fprintf(stderr, "unlocking logfile_mutex failed\n");
		}
	}							// else

	errno = saved_errno;
	return seqnb;

}								// logg


void
messip_logg_off( void )
{
	is_logg = 1;
}								// messip_logg_off


void
messip_logg_on( void )
{
	is_logg = 0;
}								// messip_logg_on
