// -------------------------------------------------------------------------
//                                  y_spawn_process.cc
//
// Proces do spawn'owania lokalnie i po sieci, oraz funkcja do takiego spawnowania // by Y
// Potrzebny ze wzgledu na przedziwne dzialanie oryginalnego spawna (klopoty z chroot())
//
// Ostatnia modyfikacja: pazdziernik 2004
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>
#include <strings.h>
#include <sys/utsname.h>
#if defined(__QNXNTO__)
#include <sys/sched.h>
#include <process.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <spawn.h>
#include <sys/netmgr.h>
#endif /* __QNXNTO__ */

#include "lib/y_spawn.h"


/* Przechwycenie sygnalu */
void catch_signal(int sig)
{
	switch (sig)
	{
	case SIGTERM :
		//     ClockPeriod(CLOCK_REALTIME, &old_cp, NULL, 0);
		//     master->msg->message("EDP terminated");
		_exit(EXIT_SUCCESS);
		break;
	case SIGSEGV:
		fprintf(stderr, "Segmentation fault in spawn process\n");
		signal(SIGSEGV, SIG_DFL);
		break;
	} // end: switch
}



int main(int argc, char *argv[], char **arge)
{

	signal(SIGTERM, &catch_signal);
	signal(SIGSEGV, &catch_signal);


	int process_state=0; // okresla stan procesu: 0 -przed komunikacja, 1 - po komunikacji
	name_attach_t *my_attach;	// by Y
	mrrocpp::lib::my_data_t msg;
	int rcvid;
	//printf("spawn_prc 1\n");


	// sprawdzenie liczby argumentow
	if (argc < 2)
	{
		printf(" Usage: y_spawn <serwer_name> \n");
		exit(EXIT_FAILURE);
	}
	//printf("spawn_prc 2\n");
	// zalozenie kanalu komunikacyjnego
	if ((my_attach = name_attach(NULL, argv[1], 0)) == NULL)
	{
		perror("Failed to attach Start pulse chanel for y_spawn\n");
	}

	while (1)
	{
		//printf("spawn_prc 3\n");

		rcvid = MsgReceive(my_attach->chid, &msg, sizeof(msg), NULL);

		if (rcvid == -1) {/* Error condition, exit */
			perror("blad receive w spawn_process\n");
			break;
		}

		if (rcvid == 0) /* Pulse received */
		{
			switch (msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				ConnectDetach(msg.hdr.scoid);
				if (process_state==1)
				{

					_exit(EXIT_SUCCESS);
				}
				break;
			case _PULSE_CODE_UNBLOCK:
				break;
			default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (msg.hdr.type >= _IO_BASE && msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		if (msg.msg_type==1) {// wiadomosc od procesu macierzystrego

			mrrocpp::lib::my_reply_data_t y_reply;
			struct utsname sysinfo;
			struct inheritance inherit;// by Y

			bzero(&inherit, sizeof(inherit));
			int liczni=0;
			char* process_name_dir_args;
			process_name_dir_args= new char[150];
			strcpy(process_name_dir_args, msg.binaries_path);// uwaga musi miec / na koncu
			strcat(process_name_dir_args, msg.program_name_and_args);

			// by Y szok
			char* argvn[51];



			char **ap = argvn;
			char *p, *val;

			/* set up inputstring */
			//printf("spawn_prc 4: %s\n", process_name_dir_args);
			for (p = process_name_dir_args; p != NULL; )
			{
				while ((val = strsep(&p, " \t")) != NULL && *val == '\0');
				*ap++ = val;
				liczni++;
			}
			*ap = 0;
			// by Y end szok !!
			//	delay(10000);

			if( uname( &sysinfo ) == -1 )
			{
				perror( "uname" );
				return EXIT_FAILURE;
			}

			if ((strcasecmp(msg.node_name, "local")==0) || (strcasecmp(msg.node_name, sysinfo.nodename)==0)) // spawn lokalny
			{
				inherit.flags = SPAWN_NOZOMBIE;
				//printf("spawn_prc 4wa\n");
			} else { // spawn po sieci

				int child_node;

				child_node = netmgr_strtond(msg.node_name, NULL);// by Y wrzucic do  robot_lptr->E_ptr->

				inherit.flags = SPAWN_SETND;
				//inherit.flags |= SPAWN_NOZOMBIE;
				//inherit.flags |= SPAWN_SETGROUP;
				inherit.flags |= SPAWN_SETSID;
				inherit.flags |= SPAWN_TCSETPGROUP;
				inherit.flags |= SPAWN_CHECK_SCRIPT;
				inherit.nd = child_node;
				//inherit.pgroup = SPAWN_NEWPGROUP;

				char node_name_with_net[PATH_MAX];

				if(netmgr_path(msg.node_name, NULL, node_name_with_net, PATH_MAX) == -1) {}



				//printf("spawn_prc 4wb: %s, %d\n", node_name_with_net, liczni);
				chroot(node_name_with_net); // zmieniamy na maszyne na ktorej ma byc uruchomiony proces
			}
			//delay(100);
			const int fd_map[] = {0, 1 ,2};
			//printf("spawn_prc 5\n");

			y_reply.pid = spawn( argvn[0], 3, fd_map, &inherit, argvn, NULL);
			//	delay(1000);
			//printf("spawn_prc 6\n");
			// by Y - powrot do starej wersji, nowa wyrzucala bledy przy wielokrotnym powolywaniu MP

			// y_reply.pid = spawn( argv[0], 0, NULL, &inherit, argv, NULL);

			if (y_reply.pid < 0) {
				//printf("spawn_prc 6a\n");
				printf ("Failed to spawn program: %s on node %s\n", argv[0], msg.node_name);
				perror("ERROR: ");
				y_reply.remote_errno = errno;
			}// end if ...pid<0
			//printf("spawn_prc 7\n");
			MsgReply(rcvid, EOK, &y_reply, sizeof(y_reply));

			process_state=1;
			//printf("spawn_prc 8\n");
		} else { // inna wiadomosc (nie od procesu macierzystego)
			/* A message (presumable ours) received, handle */
			printf("spawn server receive strange message of type: %d\n", msg.msg_type);
			MsgReply(rcvid, EOK, 0, 0);
		}
		//printf("spawn_prc 9\n");
	} // // end while
	//printf("spawn_prc 10\n");

} // end:  main( )



