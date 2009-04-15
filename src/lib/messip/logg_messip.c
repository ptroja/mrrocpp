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
#ifdef __FreeBSD__
#include <sys/uio.h>
#endif

#include "messip.h"
#include "messip_private.h"

#include "messip_utils.h"

#include "logg_messip.h"


// Updated by messip_logg_on() and messip_logg_off()
static	int	is_logg = 1;

char *logg_dir;				// Specified by --l or set to NULL

__inline__ unsigned long long int
rdtsc(void)
{
	unsigned long long int x;
	__asm__ volatile( ".byte 0x0f, 0x31" : "=A" (x) );
	return x;
}


/*
	In case the logg_mgr would be not be there!!
*/
static int
file_logg(
	FILE *output,
	logg_type_t type,
	char *text )
{
	int seqnb=0;
	unsigned long long int t0, t1, t;

	if ( logg_dir )
	{

		/*
			Try to lock the file
		*/
		char filename[PATH_MAX];
		time_t now;
		struct tm *tm;
		int fd = -1;
		int create = 0;
		struct flock lock;

		time( &now );
		tm = localtime( &now );
		sprintf( filename, "%s/%04d-%02d-%02d",
			logg_dir,
			tm->tm_year + 1900,
			tm->tm_mon + 1,
			tm->tm_mday );
		fd = open( filename, O_CREAT | O_EXCL, 0664 );
		if ( (fd == -1) && (errno == EEXIST) )
			fd = open( filename, O_RDWR );
		else
			create = 1;
		assert( fd != -1 );
		lock.l_type = F_WRLCK;
		lock.l_whence = SEEK_SET;
		lock.l_start = 0;
		lock.l_len = 0;
		fcntl( fd, F_SETLKW, &lock );

		/*
			Seq. number
		*/
		if ( create )
		{
			write( fd, "00000000 000000000000000000\015\012", 8+1+18+2 );
			lseek( fd, 0, SEEK_END );
			seqnb = 1;
			t0 = rdtsc();
			t1 = t0;
		}
		else
		{
			char temp[10];
			lseek( fd, 0, SEEK_SET );
			read( fd, temp, 8+1+18+1 );
			seqnb = atoi( temp ) + 1;
			t0 = atoll( &temp[8+1] );
			t1 = rdtsc();
			if ( text )
			{
				lseek( fd, 0, SEEK_SET );
				sprintf( temp, "%08d %018lld", seqnb, t1 );
				write( fd, temp, 8+1+18 );
				lseek( fd, 0, SEEK_END );
			}
		}

		/*
			Write the log data into the locked file
		*/
		if ( text )
		{
			char tmp[400];
			const long long int cpu_cycles = (long long int)get_cpu_clock_speed();
			char *stype;
			t = (t1-t0)/cpu_cycles;
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
			sprintf( tmp, "%6d %7lld %-15s %6d %-15s: ",
				seqnb,
				t,
				stype,
				getpid(),
				"messip_mgr" );
			write( fd, tmp, 6+1+7+1+15+1+6+1+15+1+1 );
			write( fd, text, strlen(text) );
		}							// if

		/*
			Release the lock
		*/
		lock.l_type = F_UNLCK;
		lock.l_whence = SEEK_SET;
		lock.l_start = 0;
		lock.l_len = 0;
		fcntl( fd, F_SETLKW, &lock );
		close( fd );

	}							// if ( logg_dir )

	/*
		Display also on the console ?
	*/
	if ( output )
		fputs( text, output );

	return seqnb;

}								// file_logg


int
logg(
	logg_type_t type,
	char *fmt,
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
		seqnb = file_logg( output, type, text );
	}							// if
	else
	{
		seqnb = file_logg( NULL, type, NULL );
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
