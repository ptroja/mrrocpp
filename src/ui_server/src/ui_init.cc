// ------------------------------------------------------------------------
// Proces:		UI
// Plik:			ui_init.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:
// Autor:		twiniarski/ ostatnie zmiany tkornuta
// Data:		14.03.2006
// ------------------------------------------------------------------------

/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>

#include <string.h>
#include <signal.h>
#include <dirent.h>

#include <sys/dispatch.h>

#include <fcntl.h>
#include <string.h>
#include <process.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <iostream>
#include <fstream>

#include <pthread.h>
#include <errno.h>

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui.h"
#include "lib/configurator.h"

#include "lib/messip/messip.h"

/* Local headers */
#include "proto.h"


//jk
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <sys/timeb.h>
#include <signal.h>
#include <queue>
#include <semaphore.h>

#include "ui/ui_ecp.h"

#define PORT 3490

#ifdef SA_RESTART
	#undef SA_RESTART
#endif

#define SA_RESTART 1
	pthread_t server_tid;

sem_t sem;
sem_t sem_ui;
sem_t sem_all;
sem_t sem_mp;
sem_t sem_irp6_on_track;
sem_t sem_irp6_mechatronika;
sem_t sem_irp6_postument;
sem_t sem_conveyor;
sem_t sem_speaker;

int block_mp = 0;
int block_ui = 0;
int block_all = 0;
int block_conveyor = 0;
int block_irp6_on_track = 0;
int block_irp6_postument = 0;
int block_irp6_mechatronika = 0;
int block_speaker = 0;

extern ui_robot_def ui_robot;

int rid;
int id = 0;
std::queue<Message*> q;

Message::Message(int robotId,int dialogId,int actionId,int varNum,double* v,char* m) : RobotId(robotId),DialogId(dialogId),ActionId(actionId)
{
	memset(message,'\0',256);
	message[0] = (char)(RobotId);
	message[1] = (char)(DialogId);
	message[2] = (char)(ActionId);
	if(varNum)
	{
		char Buffer[16];
		memset(Buffer,'\0',16);
		message[3] = (char)varNum;
		for(int i = 0;i < varNum;++i)
		{
			sprintf(Buffer," %d",static_cast<int>(static_cast<float>(v[i]*10000)));
			strcat(message,Buffer);
		}
		strcat(message,"\n");
	}
	else if(m)
	{
		strcat(message,m);
		strcat(message,"\n");
	}
	else strcat(message,"\n");
	if(m) delete m;
	if(v) delete v;
}

Message::~Message()
{
	// TODO: delete na tablicy??
	delete message;
}

void replySend(Message* m)
{
	sem_wait(&sem);
	q.push(m);
	sem_post(&sem);
}

void* callfunc(void* arg)
{

#ifdef PROCESS_SPAWN_RSH
	signal( SIGCHLD, &catch_signal );
#endif /* PROCESS_SPAWN_RSH */

char* Buffer = (char*)arg;
	double* v;
	int RobotId = Buffer[0];
	int DialogId = Buffer[1];
	int ActionId = Buffer[2];

	if(strlen(Buffer)>5 && Buffer[3] == 65)
	{
		int varNum = Buffer[4];
		char* pos2 = Buffer+5;
		char Buffer2[16];
		v = new double[varNum];
		for(int i = 0;i < varNum;++i)
		{
			memset(Buffer2,'\0',16);
			int val = atoi(pos2);
			v[i] = (double)val/10000.0;
			sprintf(Buffer2,"%-d",val);
			pos2+= strlen(Buffer2);
			if(val >= 0) ++pos2;
			if(pos2 >= Buffer + strlen(Buffer) && i < varNum - 1)
			{
				perror("[ERROR] Not enough variables!!");
				break;
			}
		}
	}
	else if(strlen(Buffer) > 4 && Buffer[3] == 66)
	{
		Buffer = Buffer + 4;
	}
	else Buffer = NULL;

switch(RobotId)
{
	/*RobotId = A (AllRobots)*/
	case 'A':
	{
	switch(DialogId)
	{
		/*DialogId = A (Configuration)*/
		case 'A':
		{
			sem_wait(&sem_ui);
			if(block_ui)
			{
				sem_post(&sem_ui);
				return (void*)NULL;
			}
			block_ui = 1;
			sem_post(&sem_ui);
			switch(ActionId)
			{
			/*ActionId = A (Set configuration)*/
			case 'N':
			{
				set_config(Buffer);
				printf("Duppa\n");
				break;

			}
			case 'O':
			{
				get_configs();
				break;
			}
			}
			sem_wait(&sem_ui);
			block_ui = 0;
			sem_post(&sem_ui);
			break;
		}
		/*DialogId = B (Control)*/
		case 'B':
		{
			switch(ActionId)
			{
			/*ActionId = A (Reader Start)*/
			case 'A':
			{
				sem_wait(&sem_all);
				if(block_all)
				{
					sem_post(&sem_all);
					return (void*)NULL;
				}
				block_all = 1;
				sem_post(&sem_all);
				switch((int)v[0])
				{
					case 0:	sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								pulse_reader_irp6ot_start();
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								break;
					case 1:	sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								pulse_reader_irp6p_start();
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								break;
					case 2:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								pulse_reader_conv_start();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								break;
					case 3:	sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_reader_speaker_start();
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								break;
					case 4:	sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								pulse_reader_irp6m_start();
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
					case 5:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_reader_all_robots_start();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
				}
				sem_wait(&sem_all);
				block_all = 0;
				sem_post(&sem_all);
				break;
			}
			/*ActionId = B (Reader Stop)*/
			case 'B':
			{
				sem_wait(&sem_all);
				if(block_all)
				{
					sem_post(&sem_all);
					return (void*)NULL;
				}
				block_all = 1;
				sem_post(&sem_all);
				switch((int)v[0])
				{
					case 0:	sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								pulse_reader_irp6ot_stop();
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								break;
					case 1:	sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								pulse_reader_irp6p_stop();
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								break;
					case 2:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								pulse_reader_conv_stop();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								break;
					case 3:	sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_reader_speaker_stop();
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								break;
					case 4:	sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								pulse_reader_irp6m_stop();
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
					case 5:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_reader_all_robots_stop();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
				}
				sem_wait(&sem_all);
				block_all = 0;
				sem_post(&sem_all);
				break;
			}
			/*ActionId = C (Reader Trigger)*/
			case 'C':
			{
				sem_wait(&sem_all);
				if(block_all)
				{
					sem_post(&sem_all);
					return (void*)NULL;
				}
				block_all = 1;
				sem_post(&sem_all);
				switch((int)v[0])
				{
					case 0:	sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								pulse_reader_irp6ot_trigger();
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								break;
					case 1:	sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								pulse_reader_irp6p_trigger();
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								break;
					case 2:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								pulse_reader_conv_trigger();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								break;
					case 3:	sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_reader_speaker_trigger();
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								break;
					case 4:	sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								pulse_reader_irp6m_trigger();
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
					case 5:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_reader_all_robots_trigger();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
				}
				sem_wait(&sem_all);
				block_all = 0;
				sem_post(&sem_all);
				break;
			}
			/*ActionId = D (lib::ECP Trigger)*/
			case 'D':
			{
				sem_wait(&sem_all);
				if(block_all)
				{
					sem_post(&sem_all);
					return (void*)NULL;
				}
				block_all = 1;
				sem_post(&sem_all);
				switch((int)v[0])
				{
					case 0:	sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								pulse_ecp_irp6_on_track();
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								break;
					case 1:	sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								pulse_ecp_irp6_postument();
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								break;
					case 2:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								pulse_ecp_conveyor();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								break;
					case 3:	sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_ecp_speaker();
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								break;
					case 4:	sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								pulse_ecp_irp6_mechatronika();
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
					case 5:	sem_wait(&sem_conveyor);
								if(block_conveyor)
								{
									sem_post(&sem_conveyor);
									return (void*)NULL;
								}
								block_conveyor = 1;
								sem_post(&sem_conveyor);
								sem_wait(&sem_irp6_mechatronika);
								if(block_irp6_mechatronika)
								{
									sem_post(&sem_irp6_mechatronika);
									return (void*)NULL;
								}
								block_irp6_mechatronika = 1;
								sem_post(&sem_irp6_mechatronika);
								sem_wait(&sem_irp6_postument);
								if(block_irp6_postument)
								{
									sem_post(&sem_irp6_postument);
									return (void*)NULL;
								}
								block_irp6_postument = 1;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_on_track);
								if(block_irp6_on_track)
								{
									sem_post(&sem_irp6_on_track);
									return (void*)NULL;
								}
								block_irp6_on_track = 1;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_speaker);
								if(block_speaker)
								{
									sem_post(&sem_speaker);
									return (void*)NULL;
								}
								block_speaker = 1;
								sem_post(&sem_speaker);
								pulse_ecp_all_robots();
								sem_wait(&sem_conveyor);
								block_conveyor = 0;
								sem_post(&sem_conveyor);
								sem_wait(&sem_speaker);
								block_speaker = 0;
								sem_post(&sem_speaker);
								sem_wait(&sem_irp6_on_track);
								block_irp6_on_track = 0;
								sem_post(&sem_irp6_on_track);
								sem_wait(&sem_irp6_postument);
								block_irp6_postument = 0;
								sem_post(&sem_irp6_postument);
								sem_wait(&sem_irp6_mechatronika);
								block_irp6_mechatronika = 0;
								sem_post(&sem_irp6_mechatronika);
								break;
				}
				sem_wait(&sem_all);
				block_all = 0;
				sem_post(&sem_all);
				break;
			}
			/*ActionId = E (MP Start)*/
			case 'E':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				pulse_start_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = F (MP Stop)*/
			case 'F':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				pulse_stop_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = G (MP Trigger)*/
			case 'G':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				pulse_trigger_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = H (MP Pause)*/
			case 'H':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				pulse_pause_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = I (MP Resume)*/
			case 'I':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				pulse_resume_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = J (Signal Start)*/
			case 'J':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				signal_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = K (Signal Stop)*/
			case 'K':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				signal_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = L (Signal Pause)*/
			case 'L':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				signal_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			/*ActionId = M (Signal Resume)*/
			case 'M':
			{
				sem_wait(&sem_mp);
				if(block_mp)
				{
					sem_post(&sem_mp);
					return (void*)NULL;
				}
				block_mp = 1;
				sem_post(&sem_mp);
				signal_mp();
				sem_wait(&sem_mp);
				block_mp = 0;
				sem_post(&sem_mp);
				break;
			}
			}
			sem_wait(&sem_ui);
			if(block_ui)
			{
				sem_post(&sem_ui);
				return (void*)NULL;
			}
			block_ui = 1;
			sem_post(&sem_ui);
			process_control_window_init();
			sem_wait(&sem_ui);
			block_ui = 0;
			sem_post(&sem_ui);
			break;
		}
		/*DialogId = C  (Unload All)*/
		case 'C':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				unload_all();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = D (Slay All)*/
		case 'D':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				slay_all();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = E (MP Load)*/
		case 'E':
		{
			sem_wait(&sem_mp);
			if(block_mp)
			{
				sem_post(&sem_mp);
				return (void*)NULL;
			}
			block_mp = 1;
			sem_post(&sem_mp);
			switch(ActionId)
			{
			case 'A':
			{
				MPup();
				break;
			}
			}
			sem_wait(&sem_mp);
			block_mp = 0;
			sem_post(&sem_mp);
			break;
		}
		/*DialogId = F (MP Unload)*/
		case 'F':
		{
			sem_wait(&sem_mp);
			if(block_mp)
			{
				sem_post(&sem_mp);
				return (void*)NULL;
			}
			block_mp = 1;
			sem_post(&sem_mp);
			switch(ActionId)
			{
			case 'A':
			{
				MPslay();
				break;
			}
			}
			sem_wait(&sem_mp);
			block_mp = 0;
			sem_post(&sem_mp);
			break;
		}
		/*DialogId = G (lib::EDP Load)*/
		case 'G':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				EDP_all_robots_create();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = H (lib::EDP Unload)*/
		case 'H':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				EDP_all_robots_slay();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = I (Synchronisation)*/
		case 'I':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				EDP_all_robots_synchronise();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = J (Synchro Position)*/
		case 'J':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_synchro_position();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = K (Position 0)*/
		case 'K':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_position0();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = L (Position 1)*/
		case 'L':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_position1();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = M (Position 2)*/
		case 'M':
		{
			sem_wait(&sem_all);
			if(block_all)
			{
				sem_post(&sem_all);
				return (void*)NULL;
			}
			block_all = 1;
			sem_post(&sem_all);
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_position2();
				break;
			}
			}
			sem_wait(&sem_all);
			block_all = 0;
			sem_post(&sem_all);
			break;
		}
		/*DialogId = M (UIReply)*/
		case 'N':
		{
			sem_wait(&sem_ui);
			if(block_ui)
			{
				sem_post(&sem_ui);
				return (void*)NULL;
			}
			block_ui = 1;
			sem_post(&sem_ui);
			switch(ActionId)
			{
			case 'P':
			{
				//Yes/No Answer
				yes_no_callback(v);
				break;
			}
			case 'R':
			{
				//InputDoubleAnswer
				input_double_callback(v);
				break;
			}
			case 'S':
			{
				//InputIntegerAnswer
				input_integer_callback(v);
				break;
			}
			}
			sem_wait(&sem_ui);
			block_ui = 0;
			sem_post(&sem_ui);
			break;
		}
		/*DialogId = P (FileSelection)*/
		case 'P':
		{
			sem_wait(&sem_ui);
			if(block_ui)
			{
				sem_post(&sem_ui);
				return (void*)NULL;
			}
			block_ui = 1;
			sem_post(&sem_ui);
			switch(ActionId)
			{
			/*ActionId = A (Set configuration)*/
			case 'T':
			{
				file_selection_window_send_location(Buffer);
				break;
			}
			case 'O':
			{
				get_contents(Buffer);
				break;
			}
			}
			sem_wait(&sem_ui);
			block_ui = 0;
			sem_post(&sem_ui);
			break;
		}
		/*DialogId = R (TeachingWindow)*/
		case 'O':
		{
			sem_wait(&sem_ui);
			if(block_ui)
			{
				sem_post(&sem_ui);
				return (void*)NULL;
			}
			block_ui = 1;
			sem_post(&sem_ui);
			switch(ActionId)
			{
			case 'U':
			{
				//SendMoveAnswer
				teaching_window_send_move(v);
				break;
			}
			case 'V':
			{
				//EndMotionAnswer
				teaching_window_end_motion();
				break;
			}
			}
			sem_wait(&sem_ui);
			block_ui = 0;
			sem_post(&sem_ui);
			break;
		}
		}
		break;
	}
	/*RobotId = B (IRP6 On Track)*/
	case 'B':
	{
		sem_wait(&sem_irp6_on_track);
		if(block_irp6_on_track == 1 || (!ui_robot.irp6_on_track && DialogId != 'J'))
		{
			sem_post(&sem_irp6_on_track);
			return (void*)NULL;
		}
		block_irp6_on_track = 1;
		sem_post(&sem_irp6_on_track);
		switch(DialogId)
		{
		/*DialogId = A (Kinematic)*/
		case 'A':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_kinematic();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_kinematic_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = B (ServoAlgorithm)*/
		case 'B':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_servo_algorithm();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_servo_algorithm_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = C (Post Angle Axis)*/
		case 'C':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_post_angle_axis();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_xyz_angle_axis_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_xyz_angle_axis_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_xyz_angle_axis_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = D (Post Euler)*/
		case 'D':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_post_euler();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_xyz_euler_zyz_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_xyz_euler_zyz_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_xyz_euler_zyz_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = E (Post Joints)*/
		case 'E':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_joints();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_int_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_int_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_int_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = F (Post Motors)*/
		case 'F':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_motors();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_inc_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_inc_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_inc_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = G (Pre Motors)*/
		case 'G':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
   				irp6ot_read_motors();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_inc_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_inc_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_inc_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = H (Tool Angle)*/
		case 'H':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_tool_angle();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_xyz_angle_axis_set_tool(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_xyz_angle_axis_set_tool(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_xyz_angle_axis_set_tool(v);
				break;
			}
			}
			break;
		}
		/*DialogId = I (Tool Euler)*/
		case 'I':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6ot_read_tool_euler();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6ot_xyz_euler_zyz_set_tool(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6ot_xyz_euler_zyz_set_tool(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6ot_xyz_euler_zyz_set_tool(v);
				break;
			}
			}
			break;
		}
		/*DialogId = J (lib::EDP Load)*/
		case 'J':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_on_track_create();
				break;
			}
			}
			break;
		}
		/*DialogId = K (lib::EDP Unload)*/
		case 'K':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_on_track_slay();
				break;
			}
			}
			break;
		}
		/*DialogId = L (Synchronisation)*/
		case 'L':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_on_track_synchronise();
				break;
			}
			}
			break;
		}
		/*DialogId = M (Synchro Position)*/
		case 'M':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6ot_move_to_synchro_position();
				break;
			}
			}
			break;
		}
		/*DialogId = N (Position 0)*/
		case 'N':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6ot_move_to_position0();
				break;
			}
			}
			break;
		}
		/*DialogId = O (Position 1)*/
		case 'O':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6ot_move_to_position1();
				break;
			}
			}
			break;
		}
		/*DialogId = P (Position 2)*/
		case 'P':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6ot_move_to_position2();
				break;
			}
			}
			break;
		}
		}
		sem_wait(&sem_irp6_on_track);
		block_irp6_on_track = 0;
		sem_post(&sem_irp6_on_track);
		break;
	}
	/*RobotId = C (IRP6 Postument)*/
	case 'C':
	{
		sem_wait(&sem_irp6_postument);
		if(block_irp6_postument ||  (!ui_robot.irp6_postument && DialogId != 'J'))
		{
			sem_post(&sem_irp6_postument);
			return (void*)NULL;
		}
		block_irp6_postument = 1;
		sem_post(&sem_irp6_postument);
		switch(DialogId)
		{
		/*DialogId = A (Kinematic)*/
		case 'A':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_kinematic();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_kinematic_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = B (ServoAlgorithm)*/
		case 'B':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_servo_algorithm();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_servo_algorithm_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = C (Post Angle Axis)*/
		case 'C':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_post_angle_axis();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_xyz_angle_axis_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_xyz_angle_axis_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_xyz_angle_axis_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = D (Post Euler)*/
		case 'D':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_post_euler();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_xyz_euler_zyz_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_xyz_euler_zyz_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_xyz_euler_zyz_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = E (Post Joints)*/
		case 'E':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_joints();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_int_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_int_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_int_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = F (Post Motors)*/
		case 'F':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_motors();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_inc_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_inc_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_inc_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = G (Pre Motors)*/
		case 'G':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_motors();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_inc_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_inc_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_inc_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = H (Tool Angle)*/
		case 'H':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_tool_angle();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_xyz_angle_axis_set_tool(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_xyz_angle_axis_set_tool(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_xyz_angle_axis_set_tool(v);
				break;
			}
			}
			break;
		}
		/*DialogId = I (Tool Euler)*/
		case 'I':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6p_read_tool_euler();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6p_xyz_euler_zyz_set_tool(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6p_xyz_euler_zyz_set_tool(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6p_xyz_euler_zyz_set_tool(v);
				break;
			}
			}
			break;
		}
		/*DialogId = J (lib::EDP Load)*/
		case 'J':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_postument_create();
				break;
			}
			}
			break;
		}
		/*DialogId = K (lib::EDP Unload)*/
		case 'K':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_postument_slay();
				break;
			}
			}
			break;
		}
		/*DialogId = L (Synchronisation)*/
		case 'L':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_postument_synchronise();
				break;
			}
			}
			break;
		}
		/*DialogId = M (Synchro Position)*/
		case 'M':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6p_move_to_synchro_position();
				break;
			}
			}
			break;
		}
		/*DialogId = N (Position 0)*/
		case 'N':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6p_move_to_position0();
				break;
			}
			}
			break;
		}
		/*DialogId = O (Position 1)*/
		case 'O':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6p_move_to_position1();
				break;
			}
			}
			break;
		}
		/*DialogId = P (Position 2)*/
		case 'P':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6p_move_to_position2();
				break;
			}
			}
			break;
		}
		}
		sem_wait(&sem_irp6_postument);
		block_irp6_postument = 0;
		sem_post(&sem_irp6_postument);
		break;
	}
	/*RobotId = D (Conveyor)*/
	case 'D':
	{
		sem_wait(&sem_conveyor);
		if(block_conveyor || (!ui_robot.conveyor && DialogId != 'J'))
		{
			sem_post(&sem_conveyor);
			return (void*)NULL;
		}
		block_conveyor = 1;
		sem_post(&sem_conveyor);
		switch(DialogId)
		{
		/*DialogId = A (Move)*/
		case 'A':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read Motors)*/
			case 'C':
			{
				conveyor_read_motors();
				break;
			}
			/*ActionId = B (Set Motors)*/
			case 'D':
			{
				conveyor_moves_move_motors(v);
				break;
			}
			/*ActionId = C (Increase Motors)*/
			case 'E':
			{
				conveyor_moves_move_motors(v);
				break;
			}
			/*ActionId = D (Decrease Motors)*/
			case 'F':
			{
				conveyor_moves_move_motors(v);
				break;
			}
			/*ActionId = E (Read Joints)*/
			case 'G':
			{
				conveyor_read_joints();
				break;
			}
			/*ActionId = F (Set Joints)*/
			case 'H':
			{
				conveyor_moves_move_joints(v);
				break;
			}
			/*ActionId = G (Increase Joints)*/
			case 'I':
			{
				conveyor_moves_move_joints(v);
				break;
			}
			/*ActionId = H (Decrease Joints)*/
			case 'J':
			{
				conveyor_moves_move_joints(v);
				break;
			}
			}
			break;
		}
		/*DialogId = B (Servo Algorithm)*/
		case 'B':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				conveyor_read_servo_algorithm();
				break;
			}
			/*ActionId = B(Set)*/
			case 'B':
			{
				conv_servo_algorithm_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = C (lib::EDP Load)*/
		case 'C':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_conveyor_create();
				break;
			}
			}
			break;
		}
		/*DialogId = D (lib::EDP Unload)*/
		case 'D':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_conveyor_slay();
				break;
			}
			}
			break;
		}
		/*DialogId = E (Synchronisation)*/
		case 'E':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_conveyor_synchronise();
				break;
			}
			}
			break;
		}
		/*DialogId = F (Synchro Position)*/
		case 'F':
		{
			switch(ActionId)
			{
			case 'A':
			{
				conveyor_move_to_synchro_position();
				break;
			}
			}
			break;
		}
		/*DialogId = G (Position 0)*/
		case 'G':
		{
			switch(ActionId)
			{
			case 'A':
			{
				conveyor_move_to_position0();
				break;
			}
			}
			break;
		}
		/*DialogId = H (Position 1)*/
		case 'H':
		{
			switch(ActionId)
			{
			case 'A':
			{
				conveyor_move_to_position1();
				break;
			}
			}
			break;
		}
		/*DialogId = I (Position 2)*/
		case 'I':
		{
			switch(ActionId)
			{
			case 'A':
			{
				conveyor_move_to_position2();
				break;
			}
			}
			break;
		}
		}
		sem_wait(&sem_conveyor);
		block_conveyor = 0;

		sem_post(&sem_conveyor);
		break;
	}
	/*RobotId = E (Speaker)*/
	case 'E':
	{
		sem_wait(&sem_speaker);
		if(block_speaker || (!ui_robot.speaker && DialogId != 'B'))
		{
			sem_post(&sem_speaker);
			return (void*)NULL;
		}
		block_speaker = 1;
		sem_post(&sem_speaker);
		switch(DialogId)
		{
		/*DialogId = A (Play)*/
		case 'A':
		{
			switch(ActionId)
			{
			/*ActionId = A (Play)*/
			case 'A':
			{
				speaker_play_exec();
				break;
			}
			/*ActionId = B (Check State)*/
			case 'B':
			{
				speaker_check_state();
				break;
			}
			}
			break;
		}
		/*DialogId = B (lib::EDP Load)*/
		case 'B':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_speaker_create();
				break;
			}
			}
			break;
		}
		/*DialogId = C (lib::EDP Unload)*/
		case 'C':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_speaker_slay();
				break;
			}
			}
			break;
		}
		/*DialogId = D (Sound 0)*/
		case 'D':
		{
			switch(ActionId)
			{
			case 'A':
			{
				speaker_preset_sound0();
				break;
			}
			}
			break;
		}
		/*DialogId = E (Sound 1)*/
		case 'E':
		{
			switch(ActionId)
			{
			case 'A':
			{
				speaker_preset_sound1();
				break;
			}
			}
			break;
		}
		/*DialogId = F (Sound 2)*/
		case 'F':
		{
			switch(ActionId)
			{
			case 'A':
			{
				speaker_preset_sound2();
				break;
			}
			}
			break;
		}
		}
		sem_wait(&sem_speaker);
		block_speaker = 0;
		sem_post(&sem_speaker);
		break;
	}
	/*RobotId = F (IRP6 Mechatronika)*/
	case 'F':
	{	sem_wait(&sem_irp6_mechatronika);
		if(block_irp6_mechatronika == 1 || (!ui_robot.irp6_mechatronika && DialogId != 'J'))
		{
			sem_post(&sem_irp6_mechatronika);
			return (void*)NULL;
		}
		block_irp6_mechatronika = 1;
		sem_post(&sem_irp6_mechatronika);
		switch(DialogId)
		{
		/*DialogId = A (Kinematic)*/
		case 'A':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_kinematic();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_kinematic_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = B (ServoAlgorithm)*/
		case 'B':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_servo_algorithm();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_servo_algorithm_set(v);
				break;
			}
			}
			break;
		}
		/*DialogId = C (Post Angle Axis)*/
		case 'C':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_post_angle_axis();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_xyz_angle_axis_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_xyz_angle_axis_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_xyz_angle_axis_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = D (Post Euler)*/
		case 'D':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_post_euler();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_xyz_euler_zyz_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_xyz_euler_zyz_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_xyz_euler_zyz_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = E (Post Joints)*/
		case 'E':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_joints();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_int_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_int_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_int_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = F (Post Motors)*/
		case 'F':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_motors();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_inc_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_inc_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_inc_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = G (Pre Motors)*/
		case 'G':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_motors();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_inc_motion(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_inc_motion(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_inc_motion(v);
				break;
			}
			}
			break;
		}
		/*DialogId = H (Tool Angle)*/
		case 'H':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_tool_angle();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_xyz_angle_axis_set_tool(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_xyz_angle_axis_set_tool(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_xyz_angle_axis_set_tool(v);
				break;
			}
			}
			break;
		}
		/*DialogId = I (Tool Euler)*/
		case 'I':
		{
			switch(ActionId)
			{
			/*ActionId = A (Read)*/
			case 'A':
			{
				irp6m_read_tool_euler();
				break;
			}
			/*ActionId = B (Set)*/
			case 'B':
			{
				irp6m_xyz_euler_zyz_set_tool(v);
				break;
			}
			/*ActionId = C (Increase)*/
			case 'C':
			{
				irp6m_xyz_euler_zyz_set_tool(v);
				break;
			}
			/*ActionId = D (Decrease)*/
			case 'D':
			{
				irp6m_xyz_euler_zyz_set_tool(v);
				break;
			}
			}
			break;
		}
		/*DialogId = J (lib::EDP Load)*/
		case 'J':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_mechatronika_create();
				break;
			}
			}
			break;
		}
		/*DialogId = K (lib::EDP Unload)*/
		case 'K':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_mechatronika_slay();
				break;
			}
			}
			break;
		}
		/*DialogId = L (Synchronisation)*/
		case 'L':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_irp6_mechatronika_synchronise();
				break;
			}
			}
			break;
		}
		/*DialogId = M (Synchro Position)*/
		case 'M':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6m_move_to_synchro_position();
				break;
			}
			}
			break;
		}
		/*DialogId = N (Position 0)*/
		case 'N':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6m_move_to_position0();
				break;
			}
			}
			break;
		}
		/*DialogId = O (Position 1)*/
		case 'O':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6m_move_to_position1();
				break;
			}
			}
			break;
		}
		/*DialogId = P (Position 2)*/
		case 'P':
		{
			switch(ActionId)
			{
			case 'A':
			{
				irp6m_move_to_position2();
				break;
			}
			}
			break;
 		}
		}
		sem_wait(&sem_irp6_mechatronika);
		block_irp6_mechatronika = 0;
		sem_post(&sem_irp6_mechatronika);
		break;
	default:
	{
		replySend(new Message(RobotId,DialogId,ActionId,0,NULL,NULL));
		break;
	}
	}
}

	return (void*)NULL;
}


void* reply_thread(void* arg)
{
	int fd = (int)arg;
	while(rid)
	{
		Message* m = NULL;
		sem_wait(&sem);
		if(!q.empty())
		{
			m = q.front();
			q.pop();
		}
		sem_post(&sem);
		if(m)
		{
			send(fd,m->message,strlen(m->message),0);
//			if((m->message)[0] < 'X' ) printf("%s\n",m->message);
			//TODO:
			//delete m;
		}
		usleep(1);
	}
	while(!q.empty()) q.pop();
	pthread_exit(NULL);

}


void* server_thread(void*)
{
	int sockfd,new_fd;
	struct sockaddr_in my_addr;
	struct sockaddr_in their_addr;
	socklen_t sin_size;
	struct sigaction sa;
	int yes=1;

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		perror("socket");
		exit(1);
	}

	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
		perror("setsockopt");
		exit(1);
	}

	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(PORT);
	my_addr.sin_addr.s_addr = INADDR_ANY;
	memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

	if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof my_addr) == -1) {
		perror("bind");
		exit(1);
	}

	if (listen(sockfd, 0) == -1) {
		perror("listen");
		exit(1);
	}

	timeb start,end;
	fd_set sockets;
	while(1)
	{
		sin_size = sizeof their_addr;
		printf("[SERVER] Waiting for connection\n");
		if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size)) == -1) {
			perror("accept");
			continue;
		}

		printf("[SERVER] Got connection from %s\n",inet_ntoa(their_addr.sin_addr));

		FD_ZERO(&sockets);
		FD_SET(new_fd,&sockets);
		int selectValue;
		ftime(&start);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		while(1)
		{
			char* Buffer = new char[128];
			char length;
			int size;
			int pos;
			memset(Buffer,'\0',128);
			Buffer[127] = '\n';

			selectValue = select(FD_SETSIZE,&sockets,(fd_set*)0,(fd_set*)0,&timeout);
			if(selectValue < 0)
			{
			//	perror("select");
			//	exit(EXIT_FAILURE);
			}
			else if(selectValue == 0)
			{
				ftime(&end);
				if((end.time - start.time)*1000 + (end.millitm - start.millitm) > 1000)
				{
					break;
				}
				continue;
			}

			//printf("[SERVER] Waiting for data\n");
			if ((size = recv(new_fd,&length,1,0)) == -1) perror("recv");
			else if(!size) break;

			pos = 0;
			while(length)
			{
				//printf("[SERVER] Reading data: %d|%s\n",length,Buffer);
				if ((size = recv(new_fd,Buffer+pos,length,0)) == -1) perror("recv");
				else if(!size) break;
				pos += size;
				length -= size;
			}
			if(!size) break;
			ftime(&start);

			//printf("[SERVER] Received: %s\n",Buffer);
			pthread_t tid;
			pthread_t tid2;
			//if(id) callfunc((void*)Buffer);
			sem_wait(&sem);
			if(id)
			{
					sem_post(&sem);
					int RobotId = Buffer[0];
					int DialogId = Buffer[1];
					if(RobotId == 'X') replySend(new Message('X','X','X',0,NULL,NULL));
					else if(RobotId == 'A' && DialogId == 'Q')
					{

						exit(0);
					}
					else
					{
					pthread_attr_t  attr;
					pthread_attr_init(&attr);
					pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

					callfunc((void*)Buffer);
			//		pthread_create(&tid,&attr,callfunc,(void*)Buffer);
					}
			}
			else
			{
				id = 1;
				rid = 1;
				sem_post(&sem);
				if(send(new_fd,strcat(Buffer,"\n"),pos+1, 0) == -1) perror("send");
				pthread_create(&tid2,NULL,reply_thread,(void*)new_fd);
				manage_interface();
			}
		}
		printf("[SERVER] Connection closed - Unloading all\n");
		sem_wait(&sem);
		id = 0;
		rid = 0;
		sem_post(&sem);
		close(new_fd);
		unload_all();
	}
	close(sockfd);

	return 0;
}
//~jk

ui_sr_buffer* ui_sr_obj;

ui_ecp_buffer* ui_ecp_obj;

// ini_configs* ini_con;
lib::configurator* config;

ui_state_def ui_state;

ui_msg_def ui_msg;

std::ofstream *log_file_outfile;


#if !defined(USE_MESSIP_SRR)
void *sr_thread(void* arg)
{
	// printf("watek testowy dziala\n");		// by Y&W
	lib::sr_package_t sr_msg;
	// 	char current_line[40];
	// 	char current_line[80];
	int16_t status;
	// 	int flags=0;

	name_attach_t *attach;
	// my_data_t msg;
	int rcvid;

	if ((attach = name_attach(NULL, ui_state.sr_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
	{
		perror("BLAD SR ATTACH, przypuszczalnie nie uruchomiono gns, albo blad wczytywania konfiguracji");
		return NULL;
	}
	// printf("PO ATTACH ");
	// flushall();

	while(1)
	{

		rcvid = MsgReceive_r(attach->chid, &sr_msg, sizeof(sr_msg), NULL);

		if (rcvid < 0) /* Error condition, exit */
		{
			if (rcvid == -EINTR) {
				//fprintf(stderr, "MsgReceive_r() interrupted by signal\n");
				continue;
			}

			fprintf(stderr, "SR: Receive failed (%s)\n", strerror(-rcvid));
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		}

		if (rcvid == 0) /* Pulse received */
		{
			// printf("sr puls\n");
			switch (sr_msg.hdr.code)
			{
				case _PULSE_CODE_DISCONNECT:
					ConnectDetach(sr_msg.hdr.scoid);
				break;
				case _PULSE_CODE_UNBLOCK:
				break;
				default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (sr_msg.hdr.type >= _IO_BASE && sr_msg.hdr.type <= _IO_MAX)
		{
			//  	  printf("w SR _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
			//  MsgError(rcvid, ENOSYS);
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		MsgReply(rcvid, EOK, &status, sizeof(status));

		if (strlen(sr_msg.process_name)>1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{

			ui_sr_obj->lock_mutex();

			ui_sr_obj->writer_buf_position++;
			ui_sr_obj->writer_buf_position %= UI_SR_BUFFER_LENGHT;

			ui_sr_obj->message_buffer[ui_sr_obj->writer_buf_position]=sr_msg;

			ui_sr_obj->set_new_msg();
			ui_sr_obj->unlock_mutex();

		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}

	}

	return 0;
};
#else /* USE_MESSIP_SRR */
#warning "use messip :)"
void *sr_thread(void* arg)
{
	sr_package_t sr_msg;
	int16_t status;

	messip_channel_t *ch;
	int32_t type, subtype;
	int rcvid;

	if ((ch = messip_channel_create(NULL, ui_state.sr_attach_point, MESSIP_NOTIMEOUT, 0)) == NULL) {
		perror("messip_channel_create()");
		return NULL;
	}

	while(1)
	{

		rcvid = messip_receive(ch, &type, &subtype, &sr_msg, sizeof(sr_msg), MESSIP_NOTIMEOUT);

		if (rcvid == -1) /* Error condition, exit */
		{
			perror("SR: Receive failed");
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		} else if (rcvid < -1) {
			// ie. MESSIP_MSG_DISCONNECT
			fprintf(stderr, "ie. MESSIP_MSG_DISCONNECT\n");
			continue;
		}

		status = 0;
		messip_reply(ch, rcvid, EOK, &status, sizeof(status), MESSIP_NOTIMEOUT);

		if (strlen(sr_msg.process_name)>1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
			ui_sr_obj->lock_mutex();
			// to sie zdarza choc nie wiem dlaczego

			ui_sr_obj->writer_buf_position++;
			ui_sr_obj->writer_buf_position %= UI_SR_BUFFER_LENGHT;

			ui_sr_obj->message_buffer[ui_sr_obj->writer_buf_position]=sr_msg;

			ui_sr_obj->set_new_msg();
			ui_sr_obj->unlock_mutex();

		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}
	}

	return 0;
};
#endif /* USE_MESSIP_SRR */


void *comm_thread(void* arg) {


	name_attach_t *attach;
	// my_data_t msg;
	int rcvid;
	_msg_info info;

	bool wyjscie;

	if ((attach = name_attach(NULL, ui_state.ui_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
	{
		// XXX TODO
		// return EXIT_FAILURE;
		// printf("NIE MA ATTACHA");
	}


while(1) {
	// ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;
	rcvid = MsgReceive(attach->chid, &ui_ecp_obj->ecp_to_ui_msg, sizeof(ui_ecp_obj->ecp_to_ui_msg), &info);
	ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;
     if (rcvid == -1) {/* Error condition, exit */
   		  perror("UI: Receive failed");
		// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
         break;
     }

     if (rcvid == 0) {/* Pulse received */
 		// printf("sr puls\n");
         switch (ui_ecp_obj->ecp_to_ui_msg.hdr.code) {
         case _PULSE_CODE_DISCONNECT:
             ConnectDetach(ui_ecp_obj->ecp_to_ui_msg.hdr.scoid);
         break;
         case _PULSE_CODE_UNBLOCK:
         break;
         default:
         break;
         }
         continue;
     }

     /* A QNX IO message received, reject */
     if (ui_ecp_obj->ecp_to_ui_msg.hdr.type >= _IO_BASE && ui_ecp_obj->ecp_to_ui_msg.hdr.type <= _IO_MAX) {

        MsgReply(rcvid, EOK, 0, 0);
         continue;
     }

	if (ui_state.irp6_on_track.ecp.pid<=0) {

		ui_state.irp6_on_track.ecp.pid = info.pid;

	}

	int len;
	char* msg;

      switch ( ui_ecp_obj->ecp_to_ui_msg.ecp_message ) { // rodzaj polecenia z ECP
		//jk
		case lib::C_XYZ_ANGLE_AXIS:
        case lib::C_XYZ_EULER_ZYZ:
        case lib::C_JOINT:
		case lib::C_MOTOR:

			switch ( ui_ecp_obj->ecp_to_ui_msg.ecp_message )
	{
		case lib::C_XYZ_ANGLE_AXIS:
			switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
			{
				case lib::ROBOT_IRP6_ON_TRACK:
					replySend(new Message('7','I','A',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_POSTUMENT:
					replySend(new Message('7','I','B',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_MECHATRONIKA:
					replySend(new Message('7','I','E',0,NULL,msg));
				break;
				default:
				break;
			}
		break;
		case lib::C_XYZ_EULER_ZYZ:
			switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
			{
				case lib::ROBOT_IRP6_ON_TRACK:
					replySend(new Message('7','J','A',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_POSTUMENT:
					replySend(new Message('7','J','B',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_MECHATRONIKA:
					replySend(new Message('7','J','E',0,NULL,msg));
				break;
				default:
				break;
			}
		break;
		case lib::C_JOINT:
			switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
			{
				case lib::ROBOT_IRP6_ON_TRACK:
					replySend(new Message('7','K','A',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_POSTUMENT:
					replySend(new Message('7','K','B',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_MECHATRONIKA:
					replySend(new Message('7','K','E',0,NULL,msg));
				break;
				default:
				break;
			}
		break;
		case lib::C_MOTOR:
			switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
			{
				case lib::ROBOT_IRP6_ON_TRACK:
					replySend(new Message('7','L','A',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_POSTUMENT:
					replySend(new Message('7','L','B',0,NULL,msg));
				break;
				case lib::ROBOT_IRP6_MECHATRONIKA:
					replySend(new Message('7','L','E',0,NULL,msg));
				break;
				default:
				break;
			}
		break;
	}
		ui_ecp_obj->trywait_sem();
		ui_ecp_obj->take_sem();


		if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
		break;
		case lib::YES_NO:

			len = strlen(ui_ecp_obj->ecp_to_ui_msg.string);
			msg = new char(len+1);
			strcpy(msg,ui_ecp_obj->ecp_to_ui_msg.string);
			msg[len] = '\0';
			replySend(new Message('7','A','A',0,NULL,msg));

			ui_ecp_obj->take_sem();

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
		//~jk
          break;
        case lib::MESSAGE:

			len = strlen(ui_ecp_obj->ecp_to_ui_msg.string);
			msg = new char(len+1);
			strcpy(msg,ui_ecp_obj->ecp_to_ui_msg.string);
			msg[len] = '\0';
			replySend(new Message('7','B','A',0,NULL,msg));

			ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
          break;
        case lib::DOUBLE_NUMBER:

		len = strlen(ui_ecp_obj->ecp_to_ui_msg.string);
			msg = new char(len+1);
			strcpy(msg,ui_ecp_obj->ecp_to_ui_msg.string);
			msg[len] = '\0';
			replySend(new Message('7','C','A',0,NULL,msg));

			ui_ecp_obj->take_sem();

	        if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
		  	 	printf("Blad w UI reply\n");
	  	  	}
		break;
        case lib::INTEGER_NUMBER:

			len = strlen(ui_ecp_obj->ecp_to_ui_msg.string);
			msg = new char(len+1);
			strcpy(msg,ui_ecp_obj->ecp_to_ui_msg.string);
			msg[len] = '\0';
			replySend(new Message('7','D','A',0,NULL,msg));

			ui_ecp_obj->take_sem();

		   	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
			   	printf("Blad w UI reply\n");
		   	}
 		break;
          case lib::CHOOSE_OPTION:
			// wybor ilosci dostepnych opcji w zaleznosci od wartosci ui_ecp_obj->ecp_to_ui_msg.nr_of_options

			replySend(new Message('7','E',(char)(ui_ecp_obj->ecp_to_ui_msg.nr_of_options),0,NULL,msg));

			ui_ecp_obj->take_sem();

		    	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
			   	printf("Blad w UI reply\n");
		    	}

          break;
        case lib::LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
     //    printf("lib::LOAD_FILE\n");
		          replySend(new Message('7','G','A',0,NULL,msg));
		          ui_ecp_obj->ui_rep.reply = lib::FILE_LOADED;
		       	 ui_ecp_obj->take_sem();

		     	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
			   	printf("Blad w UI reply\n");
		    	}


          break;
        case lib::SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
		   //    printf("lib::SAVE_FILE\n");
          replySend(new Message('7','H','A',0,NULL,msg));
		 ui_ecp_obj->ui_rep.reply = lib::FILE_SAVED;
  		ui_ecp_obj->take_sem();

     	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
		   	printf("Blad w UI reply\n");
	    	}

          break;
          //~jk
	default:
          perror ("Strange ECP message\n");
	} // end: switch
}// end while

return 0;
};


/* Przechwycenie sygnalu */
void catch_signal(int sig) {
	int status;
	pid_t child_pid;
  switch(sig) {
    case SIGINT :
   		fprintf(stderr, "UI CLOSING\n");
		delay(100);// czas na sutabilizowanie sie edp
		ui_state.ui_state=2;// funcja OnTimer() dowie sie ze aplikacja ma byc zamknieta
    break;
	case SIGALRM:
		printf("SIGALRM received\n");
		break;
    case SIGSEGV:
	   fprintf(stderr, "Segmentation fault in UI process\n");
	   signal(SIGSEGV, SIG_DFL);
	   break;
	case SIGCHLD:
		printf("waitpid(...)"); fflush(stdout);
	   child_pid = waitpid(-1, &status, WNOHANG);

	   if (child_pid == -1) {
		   perror("UI: waitpid()");
	   } else if (child_pid == 0) {
		   fprintf(stderr, "UI: no child exited\n");
	   } else {
		   //fprintf(stderr, "UI: child %d...\n", child_pid);
		   if (WIFEXITED(status)) {
			   fprintf(stderr, "UI: child %d exited normally with status %d\n",
					   child_pid, WEXITSTATUS(status));
		   }
		   if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
			   if (WCOREDUMP(status)) {
				   fprintf(stderr, "UI: child %d terminated by signal %d (core dumped)\n",
					   child_pid, WTERMSIG(status));
			   }
			   else
#endif /* WCOREDUMP */
			   {
				   fprintf(stderr, "UI: child %d terminated by signal %d\n",
					   child_pid, WTERMSIG(status));
			   }
		   }
		   if (WIFSTOPPED(status)) {
			   fprintf(stderr, "UI: child %d stopped\n", child_pid);
		   }
		   if (WIFCONTINUED(status)) {
			   fprintf(stderr, "UI: child %d resumed\n", child_pid);
		   }
	   }
	   break;
	default:
	   fprintf(stderr, "UI: unknown signal (%d)\n", sig);
  } // end: switch
}



int init()

	{

	//jk
	if(sem_init(&sem,0,1) == -1 || sem_init(&sem_conveyor,0,1) == -1 || sem_init(&sem_irp6_on_track,0,1) == -1 || sem_init(&sem_irp6_postument,0,1) == -1 || sem_init(&sem_irp6_mechatronika,0,1) == -1 || sem_init(&sem_speaker,0,1) == -1 || sem_init(&sem_ui,0,1) == -1 || sem_init(&sem_mp,0,1) == -1)
	{
		perror("Unable to initialize semaphore");
		return NULL;

	}//~jk




	set_ui_state_notification(UI_N_STARTING);

	struct utsname sysinfo;
	char* cwd;
	char buff[PATH_MAX + 1];
	pthread_t ui_tid;
	pthread_t sr_tid;

	//jk

	//~jk

	signal( SIGINT, &catch_signal );// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
 	signal( SIGALRM, &catch_signal );
 	signal( SIGSEGV, &catch_signal );
#ifdef PROCESS_SPAWN_RSH
	signal( SIGCHLD, &catch_signal );
#endif /* PROCESS_SPAWN_RSH */

 	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-6);

	config = NULL;

	ui_state.ui_state=1;// ui working

	ui_state.irp6_on_track.edp.state=-1; // edp nieaktywne
	ui_state.irp6_on_track.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_on_track.ecp.trigger_fd = -1;
	ui_state.irp6_on_track.edp.section_name = "[edp_irp6_on_track]";
	ui_state.irp6_on_track.ecp.section_name = "[ecp_irp6_on_track]";
	ui_state.irp6_postument.edp.state=-1; // edp nieaktywne
	ui_state.irp6_postument.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_postument.ecp.trigger_fd = -1;
	ui_state.irp6_postument.edp.section_name = "[edp_irp6_postument]";
	ui_state.irp6_postument.ecp.section_name = "[ecp_irp6_postument]";
	ui_state.speaker.edp.state=-1; // edp nieaktywne
	ui_state.speaker.edp.last_state=-1; // edp nieaktywne
	ui_state.speaker.ecp.trigger_fd = -1;
	ui_state.speaker.edp.section_name = "[edp_speaker]";
	ui_state.speaker.ecp.section_name = "[ecp_speaker]";
	ui_state.conveyor.edp.state=-1; // edp nieaktywne
	ui_state.conveyor.edp.last_state=-1; // edp nieaktywne
	ui_state.conveyor.ecp.trigger_fd = -1;
	ui_state.conveyor.edp.section_name = "[edp_conveyor]";
	ui_state.conveyor.ecp.section_name = "[ecp_conveyor]";
	ui_state.irp6_mechatronika.edp.state=-1; // edp nieaktywne
	ui_state.irp6_mechatronika.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_mechatronika.ecp.trigger_fd = -1;
	ui_state.irp6_mechatronika.edp.section_name = "[edp_irp6_mechatronika]";
	ui_state.irp6_mechatronika.ecp.section_name = "[ecp_irp6_mechatronika]";

	ui_state.file_window_mode=FSTRAJECTORY; // uczenie
	ui_state.all_edps = UI_ALL_EDPS_NONE_EDP_LOADED;
	ui_state.mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	ui_state.mp.last_state= UI_MP_NOT_PERMITED_TO_RUN ;// mp wylaczone
	ui_state.mp.pid = -1;
	ui_state.is_task_window_open=false;// informacja czy okno zadanai jest otwarte
	ui_state.is_process_control_window_open=false;// informacja czy okno sterowania procesami jest otwarte
	ui_state.process_control_window_renew = true;
	ui_state.is_file_selection_window_open=false;
	ui_state.is_wind_irp6ot_int_open=false;
	ui_state.is_wind_irp6p_int_open=false;
	ui_state.is_wind_irp6m_int_open=false;
	ui_state.is_wind_irp6ot_inc_open=false;
	ui_state.is_wind_irp6p_inc_open=false;
	ui_state.is_wind_irp6m_inc_open=false;
	ui_state.is_wind_irp6ot_xyz_euler_zyz_open=false;
	ui_state.is_wind_irp6p_xyz_euler_zyz_open=false;
	ui_state.is_wind_irp6m_xyz_euler_zyz_open=false;
	ui_state.is_wind_irp6ot_xyz_angle_axis_open=false;
	ui_state.is_wind_irp6p_xyz_angle_axis_open=false;
	ui_state.is_wind_irp6m_xyz_angle_axis_open=false;
	ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open=false;
	ui_state.is_wind_irp6p_xyz_angle_axis_ts_open=false;
	ui_state.is_wind_irp6m_xyz_angle_axis_ts_open=false;
	ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open=false;
	ui_state.is_wind_irp6p_xyz_euler_zyz_ts_open=false;
	ui_state.is_wind_irp6m_xyz_euler_zyz_ts_open=false;
	ui_state.is_teaching_window_open=false;
	ui_state.is_wind_conveyor_moves_open=false;
	ui_state.is_wind_irp6ot_kinematic_open=false;
	ui_state.is_wind_irp6p_kinematic_open=false;
	ui_state.is_wind_irp6m_kinematic_open=false;
	ui_state.is_wind_speaker_play_open=false;

	ui_state.is_wind_irp6ot_servo_algorithm_open=false;
	ui_state.is_wind_irp6p_servo_algorithm_open=false;
	ui_state.is_wind_irp6m_servo_algorithm_open=false;
	ui_state.is_wind_conv_servo_algorithm_open=false;

	ui_state.is_mp_and_ecps_active = false;
	// ui_state.is_any_edp_active = false;


	ui_state.irp6_on_track.edp.is_synchronised = false;
	ui_state.irp6_postument.edp.is_synchronised = false;
	ui_state.conveyor.edp.is_synchronised = false;
	ui_state.speaker.edp.is_synchronised = false;
	ui_state.irp6_mechatronika.edp.is_synchronised = false;
	// ustalenie katalogow UI

    if( uname( &sysinfo ) == -1 ) {
       perror( "uname" );
    }

	cwd = getcwd( buff, PATH_MAX + 1 );
	if( cwd == NULL ) {
		perror( "Blad cwd w UI" );
	}

    ui_state.ui_node_name = sysinfo.nodename;

	ui_state.binaries_local_path = cwd;
	ui_state.mrrocpp_local_path = cwd;
	ui_state.mrrocpp_local_path.erase(ui_state.mrrocpp_local_path.length()-3);// kopiowanie lokalnej sciezki bez "bin" - 3 znaki
	ui_state.binaries_network_path = "/net/";
	ui_state.binaries_network_path += ui_state.ui_node_name;
	ui_state.binaries_network_path += ui_state.binaries_local_path;
	ui_state.binaries_network_path += "/";// wysylane jako argument do procesow potomnych (mp_m i dalej)
    // printf( "system name  : %s\n", ui_state.binaries_network_path);

	// sciezka dla okna z wyborem pliku podczas wybor trajektorii dla uczenia
	ui_state.teach_filesel_fullpath = "/net/";
	ui_state.teach_filesel_fullpath += ui_state.ui_node_name;
	ui_state.teach_filesel_fullpath += ui_state.mrrocpp_local_path;
	ui_state.teach_filesel_fullpath += "trj";
	// 	printf("abba: %s\n", ui_state.teach_filesel_fullpath);

	// sciezka dla okna z wyborem pliku z trajektoria podczas wyboru pliku konfiguracyjnego
	ui_state.config_file_fullpath = "/net/";
	ui_state.config_file_fullpath += ui_state.ui_node_name;
	ui_state.config_file_fullpath += ui_state.mrrocpp_local_path;
	ui_state.config_file_fullpath += "configs";

	// printf ("Remember to create gns server\n");

	// pierwsze zczytanie pliku konfiguracyjnego (aby pobrac nazwy dla pozostalych watkow UI)
	if (get_default_configuration_file_name()>=1) // zczytaj nazwe pliku konfiguracyjnego
	 {
		initiate_configuration();
	 	// sprawdza czy sa postawione gns's i ew. stawia je
		// uwaga serwer musi byc wczesniej postawiony
		check_gns();
	} else {
		printf ("Blad manage_default_configuration_file\n");
		return 0;
	}

	ui_sr_obj = new ui_sr_buffer();
	ui_ecp_obj = new ui_ecp_buffer();

  if (pthread_create (&sr_tid, NULL, sr_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
	 printf (" Failed to thread sr_thread\n");
  }

   if (pthread_create (&ui_tid, NULL, comm_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
   		 printf (" Failed to thread comm_thread\n");
  }

	//jk
	   if (pthread_create (&server_tid, NULL, server_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
	   		 printf (" Failed to thread server_thread\n");
	  }
	//~jk

	// Zablokowanie domyslnej obslugi sygnalu SIGINT w watkach UI_SR i UI_COMM

	sigset_t set;

	sigemptyset( &set );
	sigaddset( &set, SIGINT );
	sigaddset( &set, SIGALRM );

	if  (SignalProcmask(0, sr_tid, SIG_BLOCK, &set, NULL)==-1) {
		 perror("SignalProcmask(sr_tid)");
	}

	if  (SignalProcmask(0, ui_tid, SIG_BLOCK, &set, NULL)==-1) {
		 perror("SignalProcmask(ui_tid)");
	}

	// kolejne zczytanie pliku konfiguracyjnego
	if (get_default_configuration_file_name()==1) // zczytaj nazwe pliku konfiguracyjnego
	 {
		reload_whole_configuration();


	} else {
		printf ("Blad manage_default_configuration_file\n");
		return 0;
	}

	// inicjacja pliku z logami sr

	time_t time_of_day;
	char file_date[50];
	char log_file_with_dir[100];
	char file_name[50];

	time_of_day = time( NULL );
	strftime( file_date, 40, "%g%m%d_%H-%M-%S", localtime( &time_of_day ) );

	sprintf(file_name,"/%s_sr_log", file_date);

	// 	strcpy(file_name,"/pomiar.p");
	strcpy(log_file_with_dir, "../logs/");
	strcat(log_file_with_dir, file_name);

    log_file_outfile = new std::ofstream(log_file_with_dir, std::ios::out);

	if (!(*log_file_outfile)) {
		std::cerr << "Cannot open file: " << file_name << '\n';
		perror("because of");
	}

	manage_interface();

	while(true)
	{

		OnTimer();
		usleep(100000);
	}

	quit();

	return 0;

}

int main()
{
	init();
}
