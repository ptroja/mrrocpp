// This is the server

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

#include "messip.h"

int message_handler(messip_channel_t * ch, void * handle)
{
	int32_t type, subtype;
	int index;
	char rec_buff[256], snd_buff[256];
	static int i;

	printf("message_handler at channel \"%s\"\n", ch->name);

	index = messip_receive(ch, &type, &subtype, rec_buff, sizeof(rec_buff), MESSIP_NOTIMEOUT);
	if (index < 0) {
		switch (index)
		{
			case -1:
				perror("messip_receive()");
				break;
			case MESSIP_MSG_DISCONNECT:
				printf("MESSIP_MSG_DISCONNECT\n");
				break;
			case MESSIP_MSG_DISMISSED:
				printf("MESSIP_MSG_DISMISSED\n");
				break;
			case MESSIP_MSG_TIMEOUT:
				printf("MESSIP_MSG_TIMEOUT\n");
				exit(0);
				break;
			case MESSIP_MSG_TIMER:
				printf("MESSIP_MSG_TIMER\n");
				break;
			case MESSIP_MSG_DEATH_PROCESS:
				printf("MESSIP_MSG_DEATH_PROCESS\n");
				break;
			case MESSIP_MSG_NOREPLY:
				printf("MESSIP_MSG_NOREPLY, (datalen,datalenr) = (%d, %d)\n", ch->datalen, ch->datalenr);
				break;
			case MESSIP_MSG_CONNECTING:
				printf("MESSIP_MSG_CONNECTING\n");
				break;
			default:
				printf("MESSIP_MSG_UNKNOWN %d\n", index);
				break;
		}
		return 0;
	}

	sprintf(snd_buff, "%d", 100 + i++);
	messip_reply(ch, index, 0, (void *) snd_buff, strlen(snd_buff) + 1, MESSIP_NOTIMEOUT);

	return 0;
}

int main(int argc, char *argv[])
{
	messip_channel_t *ch1, *ch2;
	//messip_dispatch_t *dpp;

#if 0
	struct sched_param param;
	if ((param.sched_priority = sched_get_priority_max(SCHED_FIFO)) == -1)
		fprintf(stderr, "sched_get_priority_max(): %s\n", strerror(errno));
	if ((sched_setscheduler(0, SCHED_FIFO, &param)) == -1)
		fprintf(stderr, "sched_setscheduler(): %s\n", strerror(errno));
#endif

	//Create a channel, in order to receive messages on it
	ch1 = messip_channel_create(NULL, "one", MESSIP_NOTIMEOUT, 0);
	assert(ch1);

	//Create a channel, in order to receive messages on it
	//ch2 = messip_channel_create(NULL, "two", MESSIP_NOTIMEOUT, 0);
	//assert(ch2);

	//dpp = messip_dispatch_create();
	//assert(dpp);

	//messip_dispatch_attach(dpp, ch1, message_handler, NULL);
	//messip_dispatch_attach(dpp, ch2, message_handler, NULL);

	//Receive messages
	for (;;) {
#if 0
		int b = messip_dispatch_block(dpp, MESSIP_NOTIMEOUT);
		printf("messip_dispatch_block = %d\n", b);

		if (b > 0) {
			messip_dispatch_handler(dpp);
		} else if (b == 0) {
			// timeout
		} else {
			perror("messip_dispatch_block()");
			break;
		}
#else
		int32_t type, subtype;
		int index;
		char rec_buff[256], snd_buff[256];

		messip_channel_t *ch = ch1;

		index = messip_receive(ch,
				&type, &subtype, rec_buff, sizeof(rec_buff),
				       MESSIP_NOTIMEOUT);
		if (index < 0) {
		    switch(index) {
			case MESSIP_MSG_DISCONNECT:
			    printf("MESSIP_MSG_DISCONNECT\n"); break;
			case MESSIP_MSG_DISMISSED:
			    printf("MESSIP_MSG_DISMISSED\n"); break;
			case MESSIP_MSG_TIMEOUT:
			    printf("MESSIP_MSG_TIMEOUT\n"); break;
			case MESSIP_MSG_TIMER:
			    printf("MESSIP_MSG_TIMER\n"); break;
			case MESSIP_MSG_DEATH_PROCESS:
			    printf("MESSIP_MSG_DEATH_PROCESS\n"); break;
			case MESSIP_MSG_NOREPLY:
			    printf("MESSIP_MSG_NOREPLY, (datalen,datalenr) = (%d, %d)\n",
				    ch1->datalen, ch1->datalenr);
			    break;
			case MESSIP_MSG_CONNECTING:
			    printf("MESSIP_MSG_CONNECTING\n"); break;
			default:
			    printf("MESSIP_MSG_UNKNOWN %d\n", index); break;
		    }
		    continue;
		}

		//30 seconds timeout
		if (index == MESSIP_MSG_TIMEOUT)
			break;
		//printf("%d/%d <- %s\n", type, subtype, rec_buff);
		sprintf(snd_buff, "%d", 100);
		messip_reply(ch, index, 0, (void *) snd_buff, strlen(snd_buff)+1, MESSIP_NOTIMEOUT);

		printf("message_replied at channel \"%s\"\n", ch->name);
		//break;
		//messip_reply(ch, index, 0, (void *) NULL, 0, MESSIP_NOTIMEOUT);
#endif
	}

	messip_channel_delete(ch1, MESSIP_NOTIMEOUT);
	//messip_channel_delete(ch2, MESSIP_NOTIMEOUT);
	//messip_dispatch_delete(dpp);

	return 0;
}
