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


int main(int argc, char *argv[], char **arge) 
{
	int process_state=0; // okresla stan procesu: 0 -przed komunikacja, 1 - po komunikacji
	name_attach_t *my_attach;	// by Y
	my_data_t msg;
	int rcvid;
	
	// sprawdzenie liczby argumentow
	if (argc < 2)
	{
		printf(" Usage: y_spawn <serwer_name> \n");
		exit(EXIT_FAILURE);
	}
	
	// zalozenie kanalu komunikacyjnego
	if ((my_attach = name_attach(NULL, argv[1], 0)) == NULL)
	{
		perror("Failed to attach Start pulse chanel for y_spawn\n");
	}
	
	while (1)
	{
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
					if (process_state==1) _exit(EXIT_SUCCESS);
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
			
			my_reply_data_t y_reply;
			struct utsname sysinfo;
			struct inheritance inherit;// by Y
						
			bzero(&inherit, sizeof(inherit));
			
			char* process_name_dir_args;
			process_name_dir_args= new char[150];
			strcpy(process_name_dir_args, msg.binaries_path);// uwaga musi miec / na koncu
			strcat(process_name_dir_args, msg.program_name_and_args);
			
			// by Y szok
			char **ap = argv, *p, *val;
			
			/* set up inputstring */
			for (p = process_name_dir_args; p != NULL; )
			{
				while ((val = strsep(&p, " \t")) != NULL && *val == '\0');
				*ap++ = val;
			}
			*ap = 0;
			// by Y end szok !!
			
			if( uname( &sysinfo ) == -1 )
			{
				perror( "uname" );
				return EXIT_FAILURE;
			}
			
			if ((strcasecmp(msg.node_name, "local")==0) || (strcasecmp(msg.node_name, sysinfo.nodename)==0)) // spawn lokalny
			{
				inherit.flags = SPAWN_NOZOMBIE;
			} else { // spawn po sieci
				int child_node;
				
				child_node = netmgr_strtond(msg.node_name, NULL);// by Y wrzucic do  robot_lptr->E_ptr->
				
				inherit.flags = SPAWN_SETND;
				inherit.flags |= SPAWN_NOZOMBIE;
				inherit.nd = child_node;
				
				char* node_name_with_net;
				node_name_with_net = new char[50];
				
				strcpy(node_name_with_net, "/net/");// uwaga musi miec / na koncu
				strcat(node_name_with_net, msg.node_name);
				
				chroot(node_name_with_net); // zmieniamy na maszyne na ktorej ma byc uruchomiony proces
			}

			const int fd_map[] = {0, 1 ,2};
			
			y_reply.pid = spawn( argv[0], 3, fd_map, &inherit, argv, NULL);

			// by Y - powrot do starej wersji, nowa wyrzucala bledy przy wielokrotnym powolywaniu MP

			// y_reply.pid = spawn( argv[0], 0, NULL, &inherit, argv, NULL);
			
			if (y_reply.pid < 0) {
				printf ("Failed to spawn program: %s on node %s\n", argv[0], msg.node_name);
				perror("ERROR: ");
				y_reply.remote_errno = errno;
			}// end if ...pid<0
			
			MsgReply(rcvid, EOK, &y_reply, sizeof(y_reply));
			
			process_state=1;
		
		} else { // inna wiadomosc (nie od procesu macierzystego)
			/* A message (presumable ours) received, handle */
			printf("spawn server receive strange message of type: %d\n", msg.msg_type);
			MsgReply(rcvid, EOK, 0, 0);
		}
	} // // end while

} // end:  main( )
