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
#include <assert.h>
#include <limits.h>
#ifdef __FreeBSD__
#include <sys/uio.h>
#endif /* __FreeBSD__ */
#if defined(__QNXNTO__)
#include <sys/procfs.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

struct dinfo_s {
	procfs_debuginfo info;
	char pathbuffer[PATH_MAX]; /* 1st byte is
	 info.path[0] */
};
#endif /* __QNXNTO__ */

#include "messip.h"

int read_etc_messip(char *hostname, int *port_used, int *port_http_used)
{
	FILE *fp;
	char line[512], host[80], path[80];
	int len, first;
	int port, port_http;

	strcpy(hostname, "localhost");
	fp = fopen(MESSIP_ETC, "r");
	if (fp == NULL) {
		perror("open(\"" MESSIP_ETC "\")");
		return -1;
	}

	*port_used = -1;
	if (port_http_used != NULL)
		*port_http_used = -1;
	first = 1;
	while (fgets(line, sizeof(line) - 1, fp)) {

		/*--- Skip line beginning with '#' ---*/
		if (((len = strlen(line)) >= 1) && (line[len - 1] == '\n'))
			line[len - 1] = 0;
		if (!strcmp(line, "") || (line[0] == '#'))
			continue;

		sscanf(line, "%s %d %d %s", host, &port, &port_http, path);
		//		if ( !strcmp( hostname, host ) )
		{
			strcpy(hostname, host);
			*port_used = port;
			if (port_http_used != NULL)
				*port_http_used = port_http;
		}
		first = 0;

	} // while

	fclose(fp);

	if (first) {
		fprintf(stderr, "No hostname in " MESSIP_ETC "\n");
		return -1;
	}

	return 0;

} // read_etc_messip


/*
 get_taskname
 Get the Process Name, from the PID

 This is a Linux and QNXNTO only version
 */

char *
get_taskname(pid_t pid, char *pname)
{
#if defined(__QNXNTO__)
	char buf[PATH_MAX + 1], *task;
	int fd, status;
	struct dinfo_s dinfo;

	sprintf(buf, "/proc/%d/as", pid);

	if ((fd = open(buf, O_RDONLY|O_NONBLOCK)) == -1)
	{
		sprintf( pname, "?%d?", pid );
		return pname;
	}

	status = devctl(fd, DCMD_PROC_MAPDEBUG_BASE, &dinfo,
			sizeof(dinfo), NULL);
	if (status != EOK) {
		close(fd);
		sprintf( pname, "?%d?", pid );
		return pname;
	}

	/*
	 * For getting other type of information, see sys/procfs.h,
	 * sys/debug.h, and sys/dcmd_proc.h
	 */

	task = strrchr(dinfo.info.path, '/');
	if (task) {
		strncpy( pname, task+1, MESSIP_MAXLEN_TASKNAME );
	} else {
		strncpy( pname, dinfo.info.path, MESSIP_MAXLEN_TASKNAME );
	}

	pname[MESSIP_MAXLEN_TASKNAME] = 0;
	close(fd);
#else
	char fname[PATH_MAX + 1];
	FILE *fp;
	int nitems;
	unsigned rpid;
	char rname[80], *p;

	/*
	 * Parse /proc/pid/stat :
	 * Format is:
	 * pid (name) ...
	 */

	sprintf(fname, "/proc/%d/stat", pid);
	fp = fopen(fname, "r");
	if (fp == NULL) {
		sprintf(pname, "?%d?", pid);
		return pname;
	}

	nitems = fscanf(fp, "%u (%s)", &rpid, rname);
	if (nitems != 2) {
		sprintf(pname, "?%d?", pid);
		fclose(fp);
		return pname;
	}

	p = strchr(rname, ')');
	if (p)
		*p = 0;
	strncpy(pname, rname, MESSIP_MAXLEN_TASKNAME);
	pname[MESSIP_MAXLEN_TASKNAME] = 0;
	fclose(fp);
#endif
	return pname;
} // get_taskname
