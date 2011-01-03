// This is the client

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#include <stdint.h>
#include <sys/syspage.h>
#endif /* __QNXNTO__ */

#include "messip.h"

#define SEND_TIMES	10
//#define ONEWAY_MESSAGE	1

int
main(int argc, char *argv[])
{
	messip_channel_t *ch;
#if !defined(ONEWAY_MESSAGE)
	char rec_buff[80];
#endif
	char snd_buff[80];
	int32_t answer;
	int i;
	double d;
#if defined(__QNXNTO__)
	uint64_t cps, cycle1, cycle2, ncycles;
#else /*!__QNXNTO__ */
	struct timespec before, after;
#endif /* __QNXNTO__ */
	int q = 0;

#if 1
    struct sched_param param;
    if ((param.sched_priority=sched_get_priority_max(SCHED_FIFO)) == -1)
        fprintf (stderr, "sched_get_priority_max(): %s\n", strerror (errno));
    if (( sched_setscheduler(0, SCHED_FIFO, &param)) == -1)
        fprintf (stderr, "sched_setscheduler(): %s\n", strerror (errno));
#endif

#if defined(__QNXNTO__)
	/* find out how many cycles per second */
	cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
	printf( "This system has %lld cycles/sec.\n",cps );
#endif /* __QNXNTO__ */

do {
	// Locate the channel where to send message to
	ch = messip_channel_connect(NULL, (argc < 2) ? "one" : argv[1], MESSIP_NOTIMEOUT);
	assert(ch != NULL);
#if 1
	// Send messages
	for (i = 0; i < SEND_TIMES; i++) {

		sprintf(snd_buff, "%d", i);
#if defined(__QNXNTO__)
		cycle1=ClockCycles();
#else /*!__QNXNTO__ */
		clock_gettime(CLOCK_REALTIME, &before);
#endif /* __QNXNTO__ */

#if defined(ONEWAY_MESSAGE)
		messip_send(ch, 100, 200, (void *) snd_buff, strlen(snd_buff)+1,	// Type=100 Subtype=200
					&answer, NULL, -1, MESSIP_NOTIMEOUT);
#else
		messip_send(ch, 100, 200, (void *) snd_buff, strlen(snd_buff)+1,	// Type=100 Subtype=200
					&answer, rec_buff, sizeof(rec_buff), MESSIP_NOTIMEOUT);
#endif

#if defined(__QNXNTO__)
		cycle2=ClockCycles();
		ncycles=cycle2-cycle1;
		d=(double)ncycles/cps;
		usleep(1000);
#else /*!__QNXNTO__ */
		clock_gettime(CLOCK_REALTIME, &after);
		d = 
			(after.tv_sec+after.tv_nsec/1e9)
			-(before.tv_sec+before.tv_nsec/1e9);
#endif /* __QNXNTO__ */
#if defined(ONEWAY_MESSAGE)
		printf("%.9f\n", d);
#else
		printf("%s: %.9f\n", rec_buff, d);
#endif
	}
	if (i < SEND_TIMES) {
		perror("messip_send()");
	}
#endif
	messip_channel_disconnect(ch, MESSIP_NOTIMEOUT);
} while(q++ < 3);
	return 0;

}
