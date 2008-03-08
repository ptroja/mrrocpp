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

#include "messip/messip.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <Pt.h>
#include <Ph.h>

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
#include <signal.h>
#include <queue>
#include <semaphore.h>

#define PORT 3490

#ifdef SA_RESTART
	#undef SA_RESTART
#endif

#define SA_RESTART 1

sem_t sem;
int rid;
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
	char* Buffer = (char*)arg;
	double* v;
	int RobotId = Buffer[0];
	int DialogId = Buffer[1];
	int ActionId = Buffer[2];
	
	if(strlen(Buffer)>5 && Buffer[3] == 65)
	{
		int varNum = Buffer[4];
		char* pos2 = Buffer+6;
		char Buffer2[16];
		v = new double[varNum];
		for(int i = 0;i < varNum;++i)
		{
			memset(Buffer2,'\0',16);
			int val = atoi(pos2);
			v[i] = (double)val/10000.0;
			sprintf(Buffer2,"%+d",val);
			pos2+= strlen(Buffer2);
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
			switch(ActionId)
			{
			/*ActionId = A (Set configuration)*/
			case 'N':
			{
				set_config(Buffer);
				break;
			}
			case 'O':
			{
				get_configs();
				break;
			}

			}
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
				switch((int)v[0])
				{
					case 0:	pulse_reader_irp6ot_start();
								break;
					case 1:	pulse_reader_irp6p_start();
								break;
					case 2:	pulse_reader_conv_start();
								break;
					case 3:	pulse_reader_speaker_start();
								break;
					case 4:	pulse_reader_irp6m_start();
								break;
					case 5:	pulse_reader_all_robots_start();
								break;			
				}
				break;
			}
			/*ActionId = B (Reader Stop)*/
			case 'B':
			{
				switch((int)v[0])
				{
					case 0:	pulse_reader_irp6ot_stop();
								break;
					case 1:	pulse_reader_irp6p_stop();
								break;
					case 2:	pulse_reader_conv_stop();
								break;
					case 3:	pulse_reader_speaker_stop();
								break;
					case 4:	pulse_reader_irp6m_stop();
								break;
					case 5:	pulse_reader_all_robots_stop();
								break;			
				}
				break;
			}
			/*ActionId = C (Reader Trigger)*/
			case 'C':
			{
				switch((int)v[0])
				{
					case 0:	pulse_reader_irp6ot_trigger();
								break;
					case 1:	pulse_reader_irp6p_trigger();
								break;
					case 2:	pulse_reader_conv_trigger();
								break;
					case 3:	pulse_reader_speaker_trigger();
								break;
					case 4:	pulse_reader_irp6m_trigger();
								break;
					case 5:	pulse_reader_all_robots_trigger();
								break;			
				}
				break;
			}
			/*ActionId = D (ECP Trigger)*/
			case 'D':
			{
				switch((int)v[0])
				{
					case 0:	pulse_ecp_irp6_on_track();
								break;
					case 1:	pulse_ecp_irp6_postument();
								break;
					case 2:	pulse_ecp_conveyor();
								break;
					case 3:	pulse_ecp_speaker();
								break;
					case 4:	pulse_ecp_irp6_mechatronika();
								break;
					case 5:	pulse_ecp_all_robots();
								break;			
				}
				break;
			}	
			/*ActionId = E (MP Start)*/
			case 'E':
			{
				pulse_start_mp();
				break;
			}
			/*ActionId = F (MP Stop)*/
			case 'F':
			{
				pulse_stop_mp();
				break;
			}
			/*ActionId = G (MP Trigger)*/
			case 'G':
			{
				pulse_trigger_mp();
				break;
			}
			/*ActionId = H (MP Pause)*/
			case 'H':
			{
				pulse_pause_mp();
				break;
			}
			/*ActionId = I (MP Resume)*/
			case 'I':
			{
				pulse_resume_mp();
				break;
			}
			/*ActionId = J (Signal Start)*/
			case 'J':
			{
				signal_mp();
				break;
			}
			/*ActionId = K (Signal Stop)*/
			case 'K':
			{
				signal_mp();
				break;
			}
			/*ActionId = L (Signal Pause)*/
			case 'L':
			{
				signal_mp();
				break;
			}
			/*ActionId = M (Signal Resume)*/
			case 'M':
			{
				signal_mp();
				break;
			}
			}
			process_control_window_init();
			break;
		}
		/*DialogId = C  (Unload All)*/
		case 'C':
		{
			switch(ActionId)
			{
			case 'A':
			{
				unload_all();
				break;
			}
			}
			break;
		}
		/*DialogId = D (Slay All)*/
		case 'D':
		{
			switch(ActionId)
			{
			case 'A':
			{
				slay_all();
				break;
			}
			}
			break;
		}
		/*DialogId = E (MP Load)*/
		case 'E':
		{
			switch(ActionId)
			{
			case 'A':
			{
				MPup();
				break;
			}
			}
			break;
		}
		/*DialogId = F (MP Unload)*/
		case 'F':
		{
			switch(ActionId)
			{
			case 'A':
			{
				MPslay();
				break;
			}
			}
			break;
		}
		/*DialogId = G (EDP Load)*/
		case 'G':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_all_robots_create();
				break;
			}
			}
			break;
		}
		/*DialogId = H (EDP Unload)*/
		case 'H':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_all_robots_slay();
				break;
			}
			}
			break;
		}
		/*DialogId = I (Synchronisation)*/
		case 'I':
		{
			switch(ActionId)
			{
			case 'A':
			{
				EDP_all_robots_synchronise();
				break;
			}
			}
			break;
		}
		/*DialogId = J (Synchro Position)*/
		case 'J':
		{
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_synchro_position();
				break;
			}
			}
			break;
		}
		/*DialogId = K (Position 0)*/
		case 'K':
		{
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_position0();
				break;
			}
			}
			break;
		}
		/*DialogId = L (Position 1)*/
		case 'L':
		{
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_position1();
				break;
			}
			}
			break;
		}
		/*DialogId = M (Position 2)*/
		case 'M':
		{
			switch(ActionId)
			{
			case 'A':
			{
				all_robots_move_to_position2();
				break;
			}
			}
			break;
		}
		/*DialogId = M (UIReply)*/
		case 'N':
		{
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
			break;
		}
		}
		break;
	}
	/*RobotId = B (IRP6 On Track)*/
	case 'B':
	{
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
		/*DialogId = J (EDP Load)*/
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
		/*DialogId = K (EDP Unload)*/
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
		break;
	}
	/*RobotId = C (IRP6 Postument)*/
	case 'C':
	{
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
		/*DialogId = J (EDP Load)*/
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
		/*DialogId = K (EDP Unload)*/
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
		break;
	}
	/*RobotId = D (Conveyor)*/
	case 'D':
	{
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
		/*DialogId = C (EDP Load)*/
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
		/*DialogId = D (EDP Unload)*/
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
		break;
	}
	/*RobotId = E (Speaker)*/
	case 'E':
	{
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
		/*DialogId = B (EDP Load)*/
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
		/*DialogId = C (EDP Unload)*/
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
		break;
	} 
	/*RobotId = F (IRP6 Mechatronika)*/
	case 'F':
	{
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
		/*DialogId = J (EDP Load)*/
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
		/*DialogId = K (EDP Unload)*/
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
		break;
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
			//delete m;
		}
		usleep(1);
	}
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

	int id = 0;
	while(1)
	{
		sin_size = sizeof their_addr;
		printf("[SERVER] Waiting for connection\n");
		if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size)) == -1) {
			perror("accept");
			continue;
		}
		printf("[SERVER] Got connection from %s\n",inet_ntoa(their_addr.sin_addr));
		while(1)
		{
			char* Buffer = new char[128];
			char length;
			int size;	
			int pos;
			memset(Buffer,'\0',128);
			Buffer[127] = '\n';
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
			//printf("[SERVER] Received: %s\n",Buffer);
			pthread_t tid;
			pthread_t tid2;
			//if(id) callfunc((void*)Buffer);
			if(id)	pthread_create(&tid,NULL,callfunc,(void*)Buffer);
			else
			{
				if(send(new_fd,strcat(Buffer,"\n"),pos+1, 0) == -1) perror("send");
				rid = 1;
				pthread_create(&tid2,NULL,reply_thread,(void*)new_fd);
				manage_interface();
			}
			id = 1;
		}
		printf("[SERVER] Connection closed\n"); 
		id = 0;
		sem_wait(&sem);
		rid = 0;
		sem_post(&sem);
		close(new_fd);
	}
	return 0;
}
//~jk


ui_sr_buffer* ui_sr_obj;
ui_ecp_buffer* ui_ecp_obj;

// ini_configs* ini_con;
configurator* config;

ui_state_def ui_state;

ui_msg_def ui_msg;

ofstream *log_file_outfile;


#if !defined(USE_MESSIP_SRR)
void *sr_thread(void* arg)
{
	// printf("watek testowy dziala\n");		// by Y&W
	sr_package sr_msg;
	// 	char current_line[40];
	// 	char current_line[80];
	word16 status;
	// 	int flags=0;
	
	name_attach_t *attach;
	// my_data_t msg;
	int rcvid;
	
	if ((attach = name_attach(NULL, ui_state.sr_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) 
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
			// 	  throw generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
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
	sr_package sr_msg;
	word16 status;

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
			perror("SR: Receive failed\n");
			// 	  throw generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
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
	_msg_info* info;
	
	info = new  _msg_info;
	
	bool wyjscie;
	
	if ((attach = name_attach(NULL, ui_state.ui_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL)
	{
		// XXX TODO
		// return EXIT_FAILURE;
		// printf("NIE MA ATTACHA");
	}
	

while(1) {
	// ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;
	rcvid = MsgReceive(attach->chid, &ui_ecp_obj->ecp_to_ui_msg, sizeof(ui_ecp_obj->ecp_to_ui_msg), NULL);
	ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;
     if (rcvid == -1) {/* Error condition, exit */
   		  perror("UI: Receive failed\n");
		// 	  throw generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
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

		ui_state.irp6_on_track.ecp.pid = info->pid;

	}

	int len;
	char* msg;
      switch ( ui_ecp_obj->ecp_to_ui_msg.ecp_message ) { // rodzaj polecenia z ECP
        case C_XYZ_ANGLE_AXIS:
        case C_XYZ_EULER_ZYZ:
        case C_JOINT:
        case C_MOTOR:
			if (ui_state.teachingstate == MP_RUNNING) {
				ui_state.teachingstate = ECP_TEACHING;
			}

			switch ( ui_ecp_obj->ecp_to_ui_msg.ecp_message )
			{
				case C_XYZ_ANGLE_AXIS:
					switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
					{
						case ROBOT_IRP6_ON_TRACK:
							//start_wnd_irp6_on_track_xyz_angle_axis (widget, apinfo, cbinfo);
							replySend(new Message('7','F','A',0,NULL,msg));
						break;
						case ROBOT_IRP6_POSTUMENT:
							//start_wnd_irp6_postument_xyz_angle_axis (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','B',0,NULL,msg));
						break;
						case ROBOT_IRP6_MECHATRONIKA:
							//start_wnd_irp6m_xyz_angle_axis (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','C',0,NULL,msg));
						break;
						default:
						break;
					}
				break;
				case C_XYZ_EULER_ZYZ:
					switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
					{
						case ROBOT_IRP6_ON_TRACK:
							//start_wnd_irp6_on_track_xyz_euler_zyz (widget, apinfo, cbinfo);
							replySend(new Message('7','F','D',0,NULL,msg));
						break;
						case ROBOT_IRP6_POSTUMENT:
							//start_wnd_irp6_postument_xyz_euler_zyz (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','E',0,NULL,msg));
						break;
						case ROBOT_IRP6_MECHATRONIKA:
							//start_wnd_irp6m_xyz_euler_zyz (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','F',0,NULL,msg));
						break;
						default:
						break;
					}
				break;
				case C_JOINT:
					switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
					{
						case ROBOT_IRP6_ON_TRACK:
							//start_wnd_irp6_on_track_int (widget, apinfo, cbinfo);
							replySend(new Message('7','F','G',0,NULL,msg));
						break;
						case ROBOT_IRP6_POSTUMENT:
							//start_wnd_irp6_postument_int (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','H',0,NULL,msg));
						break;
						case ROBOT_IRP6_MECHATRONIKA:
							//start_wnd_irp6m_int (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','I',0,NULL,msg));
						break;
						default:
						break;
					}
				break;
				case C_MOTOR:
					switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
					{
						case ROBOT_IRP6_ON_TRACK:
							//start_wnd_irp6_on_track_inc (widget, apinfo, cbinfo);
							replySend(new Message('7','F','J',0,NULL,msg));
						break;
						case ROBOT_IRP6_POSTUMENT:
							//start_wnd_irp6_postument_inc (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','K',0,NULL,msg));
						break;
						case ROBOT_IRP6_MECHATRONIKA:
							//start_wnd_irp6m_inc (widget, apinfo, cbinfo);	
							replySend(new Message('7','F','L',0,NULL,msg));
						break;
						default:
						break;
					}
				break;
			}

			ui_ecp_obj->take_sem();
			
			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
		break;
		case YES_NO:
		//jk
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
        case MESSAGE:
	
			len = strlen(ui_ecp_obj->ecp_to_ui_msg.string);
			msg = new char(len+1);
			strcpy(msg,ui_ecp_obj->ecp_to_ui_msg.string);
			msg[len] = '\0';
			replySend(new Message('7','B','A',0,NULL,msg));
		
			ui_ecp_obj->ui_rep.reply = ANSWER_YES;
 
			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
          break;
        case DOUBLE_NUMBER:
		
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
        case INTEGER_NUMBER:
		
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
          case CHOOSE_OPTION:
			// wybor ilosci dostepnych opcji w zaleznosci od wartosci ui_ecp_obj->ecp_to_ui_msg.nr_of_options
	
			replySend(new Message('7','E',(char)(ui_ecp_obj->ecp_to_ui_msg.nr_of_options),0,NULL,msg)); 
			
			ui_ecp_obj->take_sem();
		
		    	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
			   	printf("Blad w UI reply\n");
		    	}
			    	
          break;  
        case LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
     //    printf("LOAD_FILE\n");
          if (ui_state.teachingstate == MP_RUNNING) {
			
			wyjscie=false;
			while (!wyjscie)
			{
				if(!ui_state.is_file_selection_window_open)
				{
					ui_state.is_file_selection_window_open=1;
					ui_state.file_window_mode=FSTRAJECTORY;	// wybor pliku z trajektoria
					wyjscie = true;
			          PtEnter(0);
			          	ApCreateModule (ABM_file_selection_window, ABW_base, NULL);
			   	     // 	PtRealizeWidget( ABW_file_selection_window );
				  	 PtLeave(0);
				} else {
					delay(1);
				}
			}

		          ui_ecp_obj->ui_rep.reply = FILE_LOADED;
		       	 ui_ecp_obj->take_sem();
		
		     	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
			   	printf("Blad w UI reply\n");
		    	}
      
          }
          break;
        case SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
		   //    printf("SAVE_FILE\n");
          if (ui_state.teachingstate == MP_RUNNING) {
         		wyjscie = false;
			while (!wyjscie)
			{
				if(!ui_state.is_file_selection_window_open)
				{
					ui_state.is_file_selection_window_open=1;
					ui_state.file_window_mode=FSTRAJECTORY;	// wybor pliku z trajektoria
					wyjscie = true;
			          PtEnter(0);
			          ApCreateModule (ABM_file_selection_window, ABW_base, NULL);
				  	 PtLeave(0);
				} else {
					delay(1);
				}
			}
   
  		 ui_ecp_obj->ui_rep.reply = FILE_SAVED;
  		ui_ecp_obj->take_sem();

     	if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
		   	printf("Blad w UI reply\n");
	    	}
          }
          break;
	case OPEN_FORCE_SENSOR_MOVE_WINDOW:
		// obsluga sterowania silowego -> ForceSensorMove
		// przejecie kontroli nad Fotonen
		PtEnter(0);
		// stworzenie okna wndForceControl
		ApCreateModule (ABM_wndForceControl, ABW_base, NULL);
   	     // 	oddanie kontroli
	  	 PtLeave(0);
		// odeslanie -> odwieszenie ECP
		if (MsgReply(rcvid, EOK, NULL, 0)<0) {
	 		printf("Blad w UI reply\n");
			}
          break;
	case OPEN_TRAJECTORY_REPRODUCE_WINDOW:
		// obsluga odtwarzania trajektorii
		// przejecie kontroli nad Fotonen
		PtEnter(0);
		// stworzenie okna wndTrajectoryReproduce
		ApCreateModule (ABM_wndTrajectoryReproduce, ABW_base, NULL);
   	     // 	oddanie kontroli
	  	 PtLeave(0);
		// odeslanie -> odwieszenie ECP
		if (MsgReply(rcvid, EOK, NULL, 0)<0) {
	 		printf("Blad w UI reply\n");
			}
          break;
	case TR_REFRESH_WINDOW:
		// przejecie kontroli nad Fotonen
		PtEnter(0);
		// Odswiezenie okna
		TRRefreshWindow(NULL, NULL, NULL);
   	     // 	oddanie kontroli
	  	 PtLeave(0);
		// odeslanie -> odwieszenie ECP
		if (MsgReply(rcvid, EOK, NULL, 0)<0) {
	 		printf("Blad w UI reply\n");
			}
          break;
	case TR_DANGEROUS_FORCE_DETECTED:
		// przejecie kontroli nad Fotonen
		PtEnter(0);
		// Ustawienie stanu przyciskow.
		TRDangerousForceDetected(NULL, NULL, NULL);
   	     // 	oddanie kontroli
	  	 PtLeave(0);
		// odeslanie -> odwieszenie ECP
		if (MsgReply(rcvid, EOK, NULL, 0)<0) {
	 		printf("Blad w UI reply\n");
			}
          break;


	case MAM_OPEN_WINDOW:
		// Obsluga odtwarzania trajektorii.
		// Przejecie kontroli nad Fotonen.
		PtEnter(0);
		// Stworzenie okna wnd_manual_moves_automatic_measures.
//		ApCreateModule (ABM_wndTrajectoryReproduce, ABW_base, NULL);
		ApCreateModule (ABM_MAM_wnd_manual_moves_automatic_measures, ABW_base, NULL);
   	     // Oddanie kontroli.
	  	 PtLeave(0);
		// Odeslanie -> odwieszenie ECP.
		if (MsgReply(rcvid, EOK, NULL, 0)<0) {
	 		printf("Blad w UI reply\n");
			}
          break;
	case MAM_REFRESH_WINDOW:
		// Przejecie kontroli nad Photonen.
		PtEnter(0);
		// Odswiezenie okna.
		MAM_refresh_window(NULL, NULL, NULL);
   	     // 	oddanie kontroli
	  	 PtLeave(0);
		// Odeslanie -> odwieszenie ECP.
		if (MsgReply(rcvid, EOK, NULL, 0)<0) {
	 		printf("Blad w UI reply\n");
			}
          break;

	default:
          perror ("Strange ECP message\n");
	}; // end: switch
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



int init( PtWidget_t *link_instance, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
   	/* eliminate 'unreferenced' warnings */
	link_instance = link_instance, apinfo = apinfo, cbinfo = cbinfo;

	//jk
	if(sem_init(&sem,0,1) == -1)
	{
		perror("Unable to initialize semaphore\n");	
		return NULL;
		
	}//~j
	
	struct utsname sysinfo;
	char* cwd;
	char buff[PATH_MAX + 1];
	pthread_t ui_tid;
	pthread_t sr_tid;

	//jk
	pthread_t server_tid;
	//~jk
	
	signal( SIGINT, &catch_signal );// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
 	signal( SIGALRM, &catch_signal );
 	signal( SIGSEGV, &catch_signal );
#ifdef PROCESS_SPAWN_RSH
	signal( SIGCHLD, &catch_signal );
#endif /* PROCESS_SPAWN_RSH */

 	set_thread_priority(pthread_self() , MAX_PRIORITY-6);

	config = NULL;

	ui_state.ui_state=1;// ui working
	ui_state.ui_attach_point = NULL;
	ui_state.network_sr_attach_point = NULL;
	
	ui_state.irp6_on_track.edp.state=-1; // edp nieaktywne
	ui_state.irp6_on_track.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_on_track.ecp.trigger_fd = -1;
	strcpy(ui_state.irp6_on_track.edp.section_name, "[edp_irp6_on_track]");
	strcpy(ui_state.irp6_on_track.ecp.section_name, "[ecp_irp6_on_track]");
	ui_state.irp6_postument.edp.state=-1; // edp nieaktywne
	ui_state.irp6_postument.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_postument.ecp.trigger_fd = -1;
	strcpy(ui_state.irp6_postument.edp.section_name, "[edp_irp6_postument]");
	strcpy(ui_state.irp6_postument.ecp.section_name, "[ecp_irp6_postument]");
	ui_state.speaker.edp.state=-1; // edp nieaktywne
	ui_state.speaker.edp.last_state=-1; // edp nieaktywne
	ui_state.speaker.ecp.trigger_fd = -1;
	strcpy(ui_state.speaker.edp.section_name, "[edp_speaker]");
	strcpy(ui_state.speaker.ecp.section_name, "[ecp_speaker]");
	ui_state.conveyor.edp.state=-1; // edp nieaktywne
	ui_state.conveyor.edp.last_state=-1; // edp nieaktywne
	ui_state.conveyor.ecp.trigger_fd = -1;
	strcpy(ui_state.conveyor.edp.section_name, "[edp_conveyor]");
	strcpy(ui_state.conveyor.ecp.section_name, "[ecp_conveyor]");
	ui_state.irp6_mechatronika.edp.state=-1; // edp nieaktywne
	ui_state.irp6_mechatronika.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_mechatronika.ecp.trigger_fd = -1;
	strcpy(ui_state.irp6_mechatronika.edp.section_name, "[edp_irp6_mechatronika]");
	strcpy(ui_state.irp6_mechatronika.ecp.section_name, "[ecp_irp6_mechatronika]");	
		
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

    strcpy(ui_state.ui_node_name, sysinfo.nodename);

    strcpy(ui_state.binaries_local_path, cwd);
    strncpy(ui_state.mrrocpp_local_path, cwd, strlen(cwd)-3); // kopiowanie lokalnej sciezki bez "bin" - 3 znaki
    strcpy(ui_state.binaries_network_path, "/net/");
    strcat(ui_state.binaries_network_path, ui_state.ui_node_name);
    strcat(ui_state.binaries_network_path, ui_state.binaries_local_path);
    strcat(ui_state.binaries_network_path, "/");// wysylane jako argument do procesow potomnych (mp_m i dalej)
    // printf( "system name  : %s\n", ui_state.binaries_network_path);

	// sciezka dla okna z wyborem pliku podczas wybor trajektorii dla uczenia
	strcpy(ui_state.teach_filesel_fullpath, "/net/");
	strcat(ui_state.teach_filesel_fullpath, ui_state.ui_node_name);
	strcat(ui_state.teach_filesel_fullpath, ui_state.mrrocpp_local_path);
	strcat(ui_state.teach_filesel_fullpath, "trj");
	// 	printf("abba: %s\n", ui_state.teach_filesel_fullpath);

	// sciezka dla okna z wyborem pliku z trajektoria podczas wyboru pliku konfiguracyjnego
	strcpy(ui_state.config_file_fullpath, "/net/");
	strcat(ui_state.config_file_fullpath, ui_state.ui_node_name);
	strcat(ui_state.config_file_fullpath, ui_state.mrrocpp_local_path);
	strcat(ui_state.config_file_fullpath, "configs");
	
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
		PtExit( EXIT_SUCCESS );
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
		PtExit( EXIT_SUCCESS );
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
	
	log_file_outfile = new ofstream(log_file_with_dir, ios::out);
	
	if (!(*log_file_outfile)) {
		std::cerr << "Cannot open file: " << file_name << '\n';
		perror("because of");
	}

	

	manage_interface();			

	return( Pt_CONTINUE );

}

#/** PhEDIT attribute block
#-11:16777215
#0:2380:default:-3:-3:0
#2380:2427:TextFont9:-3:-3:0
#2427:51273:default:-3:-3:0
#**  PhEDIT attribute block ends (-0000170)**/
