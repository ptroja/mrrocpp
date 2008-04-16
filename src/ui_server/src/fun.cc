/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <semaphore.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <errno.h>
#include <spawn.h>
#include <process.h>
#include <assert.h>

#include "lib/srlib.h"
#include "ui/ui_const.h"
// #include "ui/ui.h"
// Konfigurator (dla PROCESS_SPAWN_RSH)
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "proto.h"

//jk
#include <dirent.h>
//~jk

extern ui_msg_def ui_msg;
extern ui_ecp_buffer* ui_ecp_obj;

extern ui_state_def ui_state;
extern configurator* config;

ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;

extern double irp6ot_current_pos[8]; // pozycja biezaca
extern double irp6ot_desired_pos[8]; // pozycja zadana

extern double irp6p_current_pos[7]; // pozycja biezaca
extern double irp6p_desired_pos[7]; // pozycja zadana

extern double irp6m_current_pos[6]; // pozycja biezaca
extern double irp6m_desired_pos[6]; // pozycja zadana

//jk

int
teaching_window_send_move(double *v)
{

	switch ( ui_ecp_obj->ecp_to_ui_msg.robot_name )
	{
		case ROBOT_IRP6_ON_TRACK:
			for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				ui_ecp_obj->ui_rep.coordinates[i] = irp6ot_current_pos[i];
		break;
		case ROBOT_IRP6_POSTUMENT:
	 		for (int i = 0; i < IRP6_POSTUMENT_NUM_OF_SERVOS; i++)
				ui_ecp_obj->ui_rep.coordinates[i] = irp6p_current_pos[i];
		break;
		case ROBOT_IRP6_MECHATRONIKA:
 			for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++)
				ui_ecp_obj->ui_rep.coordinates[i] = irp6m_current_pos[i];
		break;
		default:
		break;
	}

	ui_ecp_obj->ui_rep.double_number = v[0];
	ui_ecp_obj->ui_rep.reply = NEXT;
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	ui_ecp_obj->trywait_sem();
	ui_ecp_obj->post_sem();

	return 0;

}

int teaching_window_end_motion()
{

	ui_state.teachingstate = MP_RUNNING;
	ui_ecp_obj->ui_rep.reply = QUIT;
	
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	ui_ecp_obj->trywait_sem();
	ui_ecp_obj->post_sem();
	
	return 0;
}


int
manage_configuration_file()

	{

	reload_whole_configuration();

	set_default_configuration_file_name(); // zapis do pliku domyslnej konfiguracji

	// sprawdza czy sa postawione gns's i ew. stawia je
	// uwaga serwer musi byc wczesniej postawiony
	check_gns();

	return 0;

	}
// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu
int
manage_interface() 
{

	check_edps_state_and_modify_mp_state();

	process_control_window_init();

	// na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	replySend(new Message('A','F','A',0,NULL,NULL));



	// menu file
	// ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	// Dla robota IRP6 ON_TRACK
	manage_interface_irp6ot ();
	
	// Dla robota IRP6 POSTUMENT
	manage_interface_irp6p ();

	// Dla robota CONVEYOR
	manage_interface_conveyor ();

	// Dla robota SPEAKER
	manage_interface_speaker ();

	// Dla robota IRP6 MECHATRONIKA
	manage_interface_irp6m ();
	
	// zadanie
	// kolorowanie menu all robots


	// wlasciwosci menu  ABW_base_all_robots

	switch (ui_state.all_edps)
	{
		case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
//			printf("UI_ALL_EDPS_NONE_EDP_ACTIVATED\n");
			replySend(new Message('A','D','A',0,NULL,NULL));
		break;
		case UI_ALL_EDPS_NONE_EDP_LOADED:
//			printf("UI_ALL_EDPS_NONE_EDP_LOADED\n");
			replySend(new Message('A','D','B',0,NULL,NULL));
		break;
		case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:			
//			printf("UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED\n");
			replySend(new Message('A','D','C',0,NULL,NULL));
		break;		
		case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
//			printf("UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED\n");
			replySend(new Message('A','D','D',0,NULL,NULL));
		break;
		case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
//			printf("UI_ALL_EDPS_LOADED_AND_SYNCHRONISED\n");
		replySend(new Message('A','D','E',0,NULL,NULL));
			
			// w zaleznosci od stanu MP
			switch (ui_state.mp.state)
			{
				case UI_MP_NOT_PERMITED_TO_RUN:
					replySend(new Message('A','D','F',0,NULL,NULL));
				break;
				case UI_MP_PERMITED_TO_RUN:
						replySend(new Message('A','D','G',0,NULL,NULL));
				break;
				case UI_MP_WAITING_FOR_START_PULSE:
					replySend(new Message('A','D','H',0,NULL,NULL));
				break;		
				case UI_MP_TASK_RUNNING:
				case UI_MP_TASK_PAUSED:
					replySend(new Message('A','D','I',0,NULL,NULL));
				break;
				default:
				break;
			}			
		break;
		default:
		break;
	}

	// wlasciwosci menu task_menu
	switch (ui_state.mp.state)
	{
	
		case UI_MP_NOT_PERMITED_TO_RUN:
			replySend(new Message('A','E','A',0,NULL,NULL));
		break;
		case UI_MP_PERMITED_TO_RUN:
			replySend(new Message('A','E','B',0,NULL,NULL));
		break;
		case UI_MP_WAITING_FOR_START_PULSE:
			replySend(new Message('A','E','C',0,NULL,NULL));
		break;		
		case UI_MP_TASK_RUNNING:
		case UI_MP_TASK_PAUSED:
			replySend(new Message('A','E','D',0,NULL,NULL));
		break;
		default:
		break;
	}

	return 1;
}

int
input_double_callback(double* v)
{
	if(v[1] > 0)
	 {
		ui_ecp_obj->ui_rep.reply = ANSWER_YES;
		ui_ecp_obj->ui_rep.double_number = v[0];
	}	
	else
	{
		ui_ecp_obj->ui_rep.reply = QUIT;
		ui_ecp_obj->ui_rep.double_number = 0.0;
	}
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	ui_ecp_obj->post_sem();

	return 1;
}

int
yes_no_callback(double* v)
{
	if(v[0] > 0) ui_ecp_obj->ui_rep.reply = ANSWER_NO;
	else if(v[0] < 0) ui_ecp_obj->ui_rep.reply = QUIT;
	else ui_ecp_obj->ui_rep.reply = ANSWER_YES;
	
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	
	ui_ecp_obj->post_sem();
	
	return 1;
}

int
input_integer_callback(double* v)

{
	
	if(v[1] > 0)
	{
		ui_ecp_obj->ui_rep.reply = ANSWER_YES;
		ui_ecp_obj->ui_rep.integer_number = (int)(v[0]);
	}
	else
	{
		ui_ecp_obj->ui_rep.reply = QUIT;
		ui_ecp_obj->ui_rep.integer_number = 0;
	}
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	ui_ecp_obj->post_sem();

	return 1;
}

int
choose_option_callback(double* v)
{

	int choice = (int)(v[0]);	
	switch(choice)
	{
		case 1:
			ui_ecp_obj->ui_rep.reply = OPTION_ONE;
			break;
		case 2:
			ui_ecp_obj->ui_rep.reply = OPTION_TWO;
			break;
		case 3:
			ui_ecp_obj->ui_rep.reply = OPTION_THREE;
			break;
		case 4:
			ui_ecp_obj->ui_rep.reply = OPTION_FOUR;
			break;			
		default:
			ui_ecp_obj->ui_rep.reply = QUIT;
			break;

	}
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	ui_ecp_obj->post_sem();

	return 1;
}



int
file_selection_window_send_location(char* Buffer)
{
	char type;
	// dla pliku trajektorii
	if (Buffer !=NULL && strlen(Buffer)) {
	type = Buffer[0];
		if (ui_state.file_window_mode==FSTRAJECTORY)
		{
				strncpy(ui_ecp_obj->ui_rep.filename,rindex(Buffer,'/')+1,strlen(rindex(Buffer,'/'))-1);
				ui_ecp_obj->ui_rep.filename[strlen(rindex(Buffer,'/'))-1]='\0';
				strncpy(ui_ecp_obj->ui_rep.path,Buffer,strlen(Buffer)-strlen(rindex(Buffer,'/')));
				ui_ecp_obj->ui_rep.path[strlen(Buffer)-strlen(rindex(Buffer,'/'))]='\0';
			
			// kopiowanie biezacej sciezki, aby w nastepnym wywolaniu okna od niej zaczynac
			strcpy(ui_state.teach_filesel_fullpath, ui_ecp_obj->ui_rep.path);
			// opuszczenie semaforu dla watku UI_COMM
			ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
		}
	}
	ui_ecp_obj->post_sem();
	return(0);
}

int is_dir(char* filename)
{
	struct stat s;
	int tmp;

	tmp = stat(filename,&s);
	if(tmp == -1) return -1;
	if(S_ISREG(s.st_mode)) return 0;
	if(S_ISDIR(s.st_mode)) return 1;
	return -1;
}

int
block_all_ecp_trigger_widgets()

	{
	double* tmp = new double[1];
	tmp[0] = (ui_state.irp6_on_track.edp.is_synchronised ? 32:0)+(ui_state.irp6_postument.edp.is_synchronised ? 16:0)+(ui_state.conveyor.edp.is_synchronised ? 8:0)+(ui_state.speaker.edp.is_synchronised ? 4:0)+(ui_state.irp6_mechatronika.edp.is_synchronised ? 2:0)+1;
	
	replySend(new Message('A','H','C',1,tmp,NULL));

	return 0;
	}



int
unblock_all_ecp_trigger_widgets()

	{
	double* tmp = new double[1];
	tmp[0] = (ui_state.irp6_on_track.edp.is_synchronised ? 32:0)+(ui_state.irp6_postument.edp.is_synchronised ? 16:0)+(ui_state.conveyor.edp.is_synchronised ? 8:0)+(ui_state.speaker.edp.is_synchronised ? 4:0)+(ui_state.irp6_mechatronika.edp.is_synchronised ? 2:0)+1;

	replySend(new Message('A','H','D',1,tmp,NULL));
	return 0;
	}


int
process_control_window_init()
{
	bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

	double* tmp = new double[2];
	tmp[0] = 5;
	tmp[1] = 0;
	// Dla READER'A
	replySend(new Message('A','H','A',2,tmp,NULL));

	// Dla irp6_on_track
	process_control_window_irp6ot_section_init (wlacz_PtButton_wnd_processes_control_all_reader_start,
		wlacz_PtButton_wnd_processes_control_all_reader_stop,
		wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// Dla irp6_postument
	process_control_window_irp6p_section_init (wlacz_PtButton_wnd_processes_control_all_reader_start,
		wlacz_PtButton_wnd_processes_control_all_reader_stop,
		wlacz_PtButton_wnd_processes_control_all_reader_trigger);
	
	// Dla conveyor
	process_control_window_conveyor_section_init (wlacz_PtButton_wnd_processes_control_all_reader_start,
		wlacz_PtButton_wnd_processes_control_all_reader_stop,
		wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// Dla speakera - wylaczone	
	// Dla irp6_mechatronika
	process_control_window_irp6m_section_init (wlacz_PtButton_wnd_processes_control_all_reader_start,
		wlacz_PtButton_wnd_processes_control_all_reader_stop,
		wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	tmp = new double[2];	
	tmp[0] = 5;
	tmp[1] = (wlacz_PtButton_wnd_processes_control_all_reader_start ? 4:0)+(wlacz_PtButton_wnd_processes_control_all_reader_stop ? 2:0)+(wlacz_PtButton_wnd_processes_control_all_reader_trigger ? 1:0);
	replySend(new Message('A','H','A',2,tmp,NULL));
	 // Dla mp i ecp

if ((ui_state.mp.state!=ui_state.mp.last_state)||(ui_state.process_control_window_renew))
 {
	ui_state.process_control_window_renew = false;
	
		switch (ui_state.mp.state)
		{
			case UI_MP_PERMITED_TO_RUN:
				tmp = new double[1];
				tmp[0] = 0;
				replySend(new Message('A','H','B',1,tmp,NULL));			
				
				block_all_ecp_trigger_widgets ();
			break;
			case UI_MP_WAITING_FOR_START_PULSE:
				tmp = new double[1];
				tmp[0] = 16;
				replySend(new Message('A','H','B',1,tmp,NULL));			
				
				block_all_ecp_trigger_widgets ();
			break;
			case UI_MP_TASK_RUNNING:
				tmp = new double[1];
				tmp[0] = 13;
				replySend(new Message('A','H','B',1,tmp,NULL));			
				unblock_all_ecp_trigger_widgets ();
			break;
			case UI_MP_TASK_PAUSED:
				tmp = new double[1];
				tmp[0] = 10;
				replySend(new Message('A','H','B',1,tmp,NULL));			
				block_all_ecp_trigger_widgets ();
			break;
			default:
			break;
		}
	}
	return 0;
}

int get_configs()
{
	DIR *dp;
	struct dirent *ep;
	char* fn;	
	dp = opendir(ui_state.config_file_fullpath);
	if(dp != NULL)
	{
		while(ep = readdir(dp)) 
		{
			fn = new char[strlen(ep->d_name)+1];
			strcpy(fn,ep->d_name);
			fn[strlen(ep->d_name)] = '\0';
			if(fn[0] != '.') replySend(new Message('6','A','A',0,NULL,fn));
		}
		closedir(dp);
	}
	replySend(new Message('6','A','A',0,NULL,NULL));
}

int get_contents(char* Buffer)
{
	DIR *dp;
	struct dirent *ep;
	char* fn;	
	char* path;
	char * newBuffer = new char[strlen(ui_state.mrrocpp_local_path)+strlen(Buffer)+1];
	strcpy(newBuffer,ui_state.mrrocpp_local_path);
	strcpy(newBuffer+strlen(ui_state.mrrocpp_local_path),Buffer);
	newBuffer[strlen(ui_state.mrrocpp_local_path)+strlen(Buffer)] = '\0';
	Buffer = newBuffer;
	dp = opendir(Buffer);
	if(dp != NULL)
	{
		while(ep = readdir(dp)) 
		{
			fn = new char[strlen(ep->d_name)+2];
			path = new char[strlen(ep->d_name)+strlen(Buffer)+2];	
			strcpy(path,Buffer);
			strcpy(path+strlen(Buffer),"/");
			strcpy(path+strlen(Buffer)+1,ep->d_name);
			path[strlen(ep->d_name)+strlen(Buffer)+1] = '\0';			
			strcpy(fn+1,ep->d_name);
			if(is_dir(path) == 1) fn[0] = 'd';
			else fn[0] = 'f';
			fn[strlen(ep->d_name)+1] = '\0';
			if(fn[1] != '.') replySend(new Message('8','A','A',0,NULL,fn));
		}
		closedir(dp);
	}
	replySend(new Message('8','A','A',0,NULL,NULL));
}

int set_config(char* fn)
{

		strncpy(ui_state.config_file,fn,strlen(fn));
		ui_state.config_file[strlen(fn)]='\0';

	   manage_configuration_file() ;

		replySend(new Message('A','I','A',0,NULL,NULL));
}

int all_robots_move_to_synchro_position()
{
	if ((ui_state.mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_WAITING_FOR_START_PULSE))
	{	
		// ruch do pozcyji synchronizacji dla Irp6_on_track i dla dalszych analogicznie		
		if ((ui_state.irp6_on_track.edp.state>0) && (ui_state.irp6_on_track.edp.is_synchronised ))
			irp6ot_move_to_synchro_position();
		if ((ui_state.irp6_postument.edp.state>0) && (ui_state.irp6_postument.edp.is_synchronised ))
			irp6p_move_to_synchro_position();
		if ((ui_state.conveyor.edp.state>0) && (ui_state.conveyor.edp.is_synchronised ))
			conveyor_move_to_synchro_position();
		if ((ui_state.irp6_mechatronika.edp.state>0) && (ui_state.irp6_mechatronika.edp.is_synchronised ))
			irp6m_move_to_synchro_position();
	}
}

int all_robots_move_to_position0()
{
	if ((ui_state.mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_WAITING_FOR_START_PULSE))
	{	
		// ruch do pozcyji synchronizacji dla Irp6_on_track i dla dalszych analogicznie		
		if ((ui_state.irp6_on_track.edp.state>0) && (ui_state.irp6_on_track.edp.is_synchronised ))
			irp6ot_move_to_position0();
		if ((ui_state.irp6_postument.edp.state>0) && (ui_state.irp6_postument.edp.is_synchronised ))
			irp6p_move_to_position0();
		if ((ui_state.conveyor.edp.state>0) && (ui_state.conveyor.edp.is_synchronised ))
			conveyor_move_to_position0();
		if ((ui_state.irp6_mechatronika.edp.state>0) && (ui_state.irp6_mechatronika.edp.is_synchronised ))
			irp6m_move_to_position0();
	}
}
int all_robots_move_to_position1()
{
	if ((ui_state.mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_WAITING_FOR_START_PULSE))
	{	
		// ruch do pozcyji synchronizacji dla Irp6_on_track i dla dalszych analogicznie		
		if ((ui_state.irp6_on_track.edp.state>0) && (ui_state.irp6_on_track.edp.is_synchronised ))
			irp6ot_move_to_position1();
		if ((ui_state.irp6_postument.edp.state>0) && (ui_state.irp6_postument.edp.is_synchronised ))
			irp6p_move_to_position1();
		if ((ui_state.conveyor.edp.state>0) && (ui_state.conveyor.edp.is_synchronised ))
			conveyor_move_to_position1();
		if ((ui_state.irp6_mechatronika.edp.state>0) && (ui_state.irp6_mechatronika.edp.is_synchronised ))
			irp6m_move_to_position1();
	}
}
int all_robots_move_to_position2()
{
	if ((ui_state.mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_WAITING_FOR_START_PULSE))
	{	
		// ruch do pozcyji synchronizacji dla Irp6_on_track i dla dalszych analogicznie		
		if ((ui_state.irp6_on_track.edp.state>0) && (ui_state.irp6_on_track.edp.is_synchronised ))
			irp6ot_move_to_position2();
		if ((ui_state.irp6_postument.edp.state>0) && (ui_state.irp6_postument.edp.is_synchronised ))
			irp6p_move_to_position2();
		if ((ui_state.conveyor.edp.state>0) && (ui_state.conveyor.edp.is_synchronised ))
			conveyor_move_to_position2();
		if ((ui_state.irp6_mechatronika.edp.state>0) && (ui_state.irp6_mechatronika.edp.is_synchronised ))
			irp6m_move_to_position2();
	}
}
int EDP_all_robots_create()
{
	EDP_conveyor_create();
	EDP_speaker_create();
	EDP_irp6_on_track_create();
	EDP_irp6_mechatronika_create();
	EDP_irp6_postument_create();
}
int EDP_all_robots_slay()
{
	EDP_conveyor_slay();
	EDP_speaker_slay();
	EDP_irp6_on_track_slay();
	EDP_irp6_mechatronika_slay();
	EDP_irp6_postument_slay();
}
int MPup()
{

	set_ui_state_notification(UI_N_PROCESS_CREATION);
	char tmp_string[100];

	if (ui_state.mp.pid ==-1)
	{
		ui_state.mp.node_nr = config->return_node_number(ui_state.mp.node_name);
		
		strcpy(tmp_string, "/dev/name/global/");
		strcat(tmp_string, ui_state.mp.network_pulse_attach_point);
			
		// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny MP
		if( access(tmp_string, R_OK)== 0  )
		{
			ui_msg.ui->message("MP already exists");
			}
		else
		{
			ui_state.mp.pid = config->process_spawn("[mp]");
		
			if(ui_state.mp.pid>0) {
		
				 short tmp = 0;
			 	// kilka sekund  (~1) na otworzenie urzadzenia
				while( (ui_state.mp.pulse_fd = name_open(ui_state.mp.network_pulse_attach_point, NAME_FLAG_ATTACH_GLOBAL))  < 0 )
					if((tmp++)<20)
						delay(100);
					else{
					   printf("blad odwolania do: %s,\n", ui_state.mp.network_pulse_attach_point);
	   				   break;
					};
		
			  ui_state.teachingstate = MP_RUNNING;
			  replySend(new Message('A','A','A',0,NULL,NULL));
			  ui_state.mp.state = UI_MP_WAITING_FOR_START_PULSE; // mp wlaczone
		  	}
			else
			{
				printf("Mp spawn failed\n");
			}
		}
	}	
	manage_interface();
	return 0;
}

bool deactivate_ecp_trigger (ecp_edp_ui_robot_def& robot_l)
{

	if (robot_l.is_active)
	{
		if (robot_l.ecp.trigger_fd>=0) 
		{
			name_close(robot_l.ecp.trigger_fd);
		}
		robot_l.ecp.trigger_fd = -1;
		robot_l.ecp.pid = -1;
		return true;
	}
	
	return false;
}


int
execute_mp_pulse (char pulse_code)
{

	int ret = -2;
	long pulse_value = 1 ;

	// printf("w send pulse\n");
	if (ui_state.mp.pulse_fd>0) {
		if (ret == MsgSendPulse (ui_state.mp.pulse_fd , sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {
	
			  perror("Blad w wysylaniu pulsu do mp\n");
			   fprintf( stderr, "Blad w wysylaniu pulsu do mp error: %s \n",       strerror( errno ) );
			   delay(1000);
		}
	}
	return ret;
	
}




int MPslay()
{
	if (ui_state.mp.pid!=-1)
	{
		if ((ui_state.mp.state == UI_MP_TASK_RUNNING) || (ui_state.mp.state == UI_MP_TASK_PAUSED))
		{
			pulse_stop_mp();
		}
		name_close(ui_state.mp.pulse_fd);
		SignalKill(ui_state.mp.node_nr, ui_state.mp.pid, 0, SIGTERM, 0, 0);
		ui_state.mp.state = UI_MP_PERMITED_TO_RUN; // mp wylaczone
	}
	ui_state.mp.pid = -1;
	ui_state.mp.pulse_fd = -1;
	
	deactivate_ecp_trigger (ui_state.irp6_on_track);
	deactivate_ecp_trigger (ui_state.irp6_postument);
	deactivate_ecp_trigger (ui_state.conveyor);
	deactivate_ecp_trigger (ui_state.speaker);
	deactivate_ecp_trigger (ui_state.irp6_mechatronika);

	replySend(new Message('A','B','A',0,NULL,NULL));
	manage_interface();
	return 0;
}



int pulse_start_mp()
{
	 if (ui_state.mp.state == UI_MP_WAITING_FOR_START_PULSE)
	 {
		ui_state.mp.state = UI_MP_TASK_RUNNING;// czekanie na stop
		execute_mp_pulse (MP_START);
		process_control_window_init();
		manage_interface();
	}

	return 0;
}

int pulse_stop_mp()
{
	if ((ui_state.mp.state == UI_MP_TASK_RUNNING) || (ui_state.mp.state == UI_MP_TASK_PAUSED))
	{
		ui_state.mp.state = UI_MP_WAITING_FOR_START_PULSE;// czekanie na stop
		execute_mp_pulse (MP_STOP);
	}
manage_interface();
	return 0;
}

int pulse_pause_mp()
{
	if (ui_state.mp.state == UI_MP_TASK_RUNNING)
	{
		ui_state.mp.state = UI_MP_TASK_PAUSED;// czekanie na stop
		execute_mp_pulse (MP_PAUSE);
	}
manage_interface();
	return 0;
}

int pulse_resume_mp()
{
	if (ui_state.mp.state == UI_MP_TASK_PAUSED)
	{
		ui_state.mp.state = UI_MP_TASK_RUNNING;// czekanie na stop
		execute_mp_pulse (MP_RESUME);
	}
manage_interface();
	return 0;
}

int pulse_trigger_mp()
{
	if (ui_state.mp.state == UI_MP_TASK_RUNNING)
	{
		execute_mp_pulse (MP_TRIGGER);
	}
manage_interface();
	return 0;
}

int signal_mp()
{
	fprintf(stderr, "UI: %s:%d signal_mp() -- thish should not happend!\n", __FILE__, __LINE__);
	assert(0);

	return 0;
}

int pulse_reader_all_robots_start()
{
	pulse_reader_conv_start();
	pulse_reader_speaker_start();
	pulse_reader_irp6ot_start();
	pulse_reader_irp6m_start();
	pulse_reader_irp6p_start();
}
int pulse_reader_all_robots_stop()
{
	pulse_reader_conv_stop();
	pulse_reader_speaker_stop();
	pulse_reader_irp6ot_stop();
	pulse_reader_irp6m_stop();
	pulse_reader_irp6p_stop();
}
int pulse_reader_all_robots_trigger()
{
	pulse_reader_conv_trigger();
	pulse_reader_speaker_trigger();
	pulse_reader_irp6ot_trigger();
	pulse_reader_irp6m_trigger();
	pulse_reader_irp6p_trigger();
}
int pulse_ecp_all_robots()
{
	pulse_ecp_conveyor();
	pulse_ecp_speaker();
	pulse_ecp_irp6_on_track();
	pulse_ecp_irp6_mechatronika();
	pulse_ecp_irp6_postument();
}
int unload_all()
{
	MPslay();
	delay(200);
	EDP_all_robots_slay();
	return 0;
}

int slay_all()
{
	unload_all();
	for (std::list<program_node_def>::iterator program_node_list_iterator = ui_state.program_node_list.begin(); program_node_list_iterator != ui_state.program_node_list.end(); program_node_list_iterator++)
	{
		char system_command[100];
#if defined(PROCESS_SPAWN_RSH)
		sprintf(system_command, "rsh %s killall -e -q -v %s",
				program_node_list_iterator->node_name,
				program_node_list_iterator->program_name
			   );
#else
		sprintf(system_command, "slay -v -f -n %s %s",
				program_node_list_iterator->node_name,
				program_node_list_iterator->program_name
			   );
#endif
//		printf("aaa: %s\n", system_command);
		system(system_command);		
		
		delay(10);
						
#if defined(PROCESS_SPAWN_RSH)
		sprintf(system_command, "rsh %s killall -e -q -v %s",
				program_node_list_iterator->node_name,
				program_node_list_iterator->program_name
			   );
#else
		sprintf(system_command, "slay -9 -v -f -n %s %s",
				program_node_list_iterator->node_name,
				program_node_list_iterator->program_name
			   );
#endif
//		printf("bbb: %s\n", system_command);
		system(system_command);
	}
	
	manage_interface();
	
	return 0;
}

int EDP_all_robots_synchronise()
{
	EDP_conveyor_synchronise();
	EDP_irp6_on_track_synchronise();
	EDP_irp6_mechatronika_synchronise();
	EDP_irp6_postument_synchronise();
}

int set_ui_busy_state_notification () 
{
	set_ui_state_notification(UI_N_BUSY);
	return 0;
}

int set_ui_ready_state_notification () 
{
	set_ui_state_notification(UI_N_READY);
	return 0;
}

int set_ui_state_notification (UI_NOTIFICATION_STATE_ENUM new_notifacion)
{
	if (new_notifacion != ui_state.notification_state)
	{
		ui_state.notification_state = new_notifacion;
		
		switch (new_notifacion)
		{
			case UI_N_STARTING:
			replySend(new Message('9','A','A',0,NULL,NULL));
			break;
			case UI_N_READY:
			replySend(new Message('9','B','A',0,NULL,NULL));
			break;
			case UI_N_BUSY:
			replySend(new Message('9','C','A',0,NULL,NULL));
			break;
			case UI_N_EXITING:
			replySend(new Message('9','D','A',0,NULL,NULL));
			break;			
			case UI_N_COMMUNICATION:
			replySend(new Message('9','E','A',0,NULL,NULL));
			break;
			case UI_N_SYNCHRONISATION:
			replySend(new Message('9','F','A',0,NULL,NULL));
			break;
			case UI_N_PROCESS_CREATION:
			replySend(new Message('9','G','A',0,NULL,NULL));
			break;
		}
	return 1;
	}
	return 0;
}

int quit()
{
	int do_close=1; // czy zamykac naprawde??

	if (do_close) // jesli apliakcja ma byc zamknieta
	{

			printf("UI CLOSING\n");
			delay(100);// czas na sutabilizowanie sie edp
			ui_state.ui_state=2;// funcja OnTimer dowie sie ze aplikacja ma byc zamknieta

	}
	return 0;
}
//~jk



int clear_all_configuration_lists()
{
	// clearing of lists
	for (std::list<char*>::iterator list_iterator = ui_state.section_list.begin(); list_iterator != ui_state.section_list.end(); list_iterator++)
	{
		delete *list_iterator;
	}	
	ui_state.section_list.clear();
	
	for (std::list<char*>::iterator node_list_iterator = ui_state.config_node_list.begin(); node_list_iterator != ui_state.config_node_list.end(); node_list_iterator++)
	{
		delete *node_list_iterator;
	}
	ui_state.config_node_list.clear();
	
	for (std::list<char*>::iterator node_list_iterator = ui_state.all_node_list.begin(); node_list_iterator != ui_state.all_node_list.end(); node_list_iterator++)
	{
		delete *node_list_iterator;
	}
	ui_state.all_node_list.clear();
	
	for (std::list<program_node_def>::iterator program_node_list_iterator = ui_state.program_node_list.begin(); program_node_list_iterator != ui_state.program_node_list.end(); program_node_list_iterator++)
	{
		delete program_node_list_iterator->program_name;
		delete program_node_list_iterator->node_name;
	}
	ui_state.program_node_list.clear();

}



int initiate_configuration() 
{
	
	char* tmp;
	
	bool wyjscie = false;
	 
 	 if (access(ui_state.config_file_relativepath, R_OK)!= 0 ) 
	{
	 	printf ("Wrong entry in default_file.cfg - load another configuration than: %s\n", ui_state.config_file_relativepath);
		strcpy(ui_state.config_file_relativepath,"../configs/common.ini");
	
	 }

	// sprawdzenie czy nazwa sesji jest unikalna

	while (!wyjscie)
	{	

        time_t time_of_day;
        char file_date[50];
        char log_file_with_dir[100];
        char file_name[50];

		char* tmp;

        time_of_day = time( NULL );
        strftime( ui_state.session_name, 8, "_%H%M%S", localtime( &time_of_day ) );
        
        if (config) delete config;
		config = new configurator(ui_state.ui_node_name, ui_state.mrrocpp_local_path, ui_state.config_file, "[ui]", 
		ui_state.session_name);
		
		tmp = config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");

		// wykrycie identyczneych nazw sesji
		wyjscie = true;
		DIR* dirp;
	    struct dirent* direntp;
	
	    dirp = opendir( "/dev/name/global" );
	    if( dirp != NULL ) {
	        for(;;) {
	            direntp = readdir( dirp );
	            if( direntp == NULL ) break;
	
	            // printf( "%s\n", direntp->d_name );
	            if  (strcmp(direntp->d_name, tmp) == 0)
	            {
	            	wyjscie = false;
	            }
	        }
	
	        closedir( dirp );
	 
	    }
	    
	    delete[] tmp;
		
	}
		
	ui_state.ui_attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "ui_attach_point", "[ui]");
	ui_state.sr_attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	ui_state.network_sr_attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	
	clear_all_configuration_lists();

	// sczytanie listy sekcji
	fill_section_list (ui_state.config_file_relativepath);
	fill_section_list ("../configs/common.ini");
	fill_node_list();
	fill_program_node_list();

		
	return 1;
}



int 
reload_whole_configuration() {

	char sr_msg_buf[100];

 	 if (access(ui_state.config_file_relativepath, R_OK) != 0 ) 
	{
	 	printf ("Wrong entry in default_file.cfg - load another configuration than: %s\n", ui_state.config_file_relativepath);
		strcpy(ui_state.config_file_relativepath,"../configs/common.ini");
		 }	

 
	if ((ui_state.mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (ui_state.mp.state == UI_MP_PERMITED_TO_RUN) ){ // jesli nie dziala mp podmien mp ecp vsp
	
		config->change_ini_file (ui_state.config_file);
	
		ui_state.is_mp_and_ecps_active = config->return_int_value("is_mp_and_ecps_active");
	
		switch (ui_state.all_edps)
		{
			case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
			case UI_ALL_EDPS_NONE_EDP_LOADED:
			
				// dla robota irp6 on_track			
				reload_irp6ot_configuration ();
		
				// dla robota irp6 postument	
				reload_irp6p_configuration ();
				
				// dla robota conveyor
				reload_conveyor_configuration ();
		
				// dla robota speaker
				reload_speaker_configuration ();
				
				// dla robota irp6 mechatronika	
				reload_irp6m_configuration ();
			break;
			default:
			break;
		}
				

		// clearing of lists
		clear_all_configuration_lists();
		
		// sczytanie listy sekcji
		fill_section_list (ui_state.config_file_relativepath);
		fill_section_list ("../configs/common.ini");
		fill_node_list();
		fill_program_node_list();
		
	
		//ui_state.section_list.clear();
		// ui_state.section_list.push_front("ala");
		//  ui_state.section_list.push_front("bala");
		//   ui_state.section_list.push_front("mala");
		//	printf("bbb: %d\n", ui_state.section_list.size());
		/*
			for (list<char*>::iterator list_iterator = ui_state.section_list.begin(); list_iterator != ui_state.section_list.end(); list_iterator++)
			{
				printf("section_name: %s\n", *list_iterator);
			
			}
		
			for (list<char*>::iterator node_list_iterator = ui_state.node_list.begin(); node_list_iterator != ui_state.node_list.end(); node_list_iterator++)
			{
				printf("node_name: %s\n", *node_list_iterator);
			}
			
			for (list<program_node_def>::iterator program_node_list_iterator = ui_state.program_node_list.begin(); program_node_list_iterator != ui_state.program_node_list.end(); program_node_list_iterator++)
			{
				printf("node_name: %s\n", program_node_list_iterator->node_name);
			}
			*/
		
		// zczytanie konfiguracji UI
			
	
		// zczytanie konfiguracji MP
		
		if (ui_state.is_mp_and_ecps_active) 
		{
		
			delete [] ui_state.mp.network_pulse_attach_point;
			ui_state.mp.network_pulse_attach_point = config->return_attach_point_name	(configurator::CONFIG_SERVER, "mp_pulse_attach_point", "[mp]");
			
			delete [] ui_state.mp.node_name;
			ui_state.mp.node_name = config->return_string_value ("node_name", "[mp]");

			ui_state.mp.pid = -1;
		}
		
		// inicjacja komunikacji z watkiem sr
		if (ui_msg.ui == NULL)
		{
			if ((ui_msg.ui = new sr_ui(UI, ui_state.ui_attach_point, ui_state.network_sr_attach_point)) == NULL) {
				perror ( "Unable to locate SR\n");
			} else {
				ui_msg.ui->message("started");	
			}
		}

		// inicjacja komunikacji z watkiem sr
		if (ui_msg.all_ecp == NULL)
		{
			if ((ui_msg.all_ecp = new sr_ecp(ECP, "ui_all_ecp" , ui_state.network_sr_attach_point)) == NULL) {
				perror ( "Unable to locate SR\n");
			} else {
					
			}
		}

		// wypisanie komunikatu o odczytaniu konfiguracji
		if (ui_msg.ui)
		{
			strcpy(sr_msg_buf, ui_state.config_file);
			strcat(sr_msg_buf, " config file loaded");
			ui_msg.ui->message(sr_msg_buf);
		}
	
	}

	manage_interface();
	
	return 1;
}



// fills section list of configuration files
int fill_section_list(char* file_name_and_path)
{

	static char line[256];
	// char program_name[50];
	// char node_name[50];
	
	FILE *file;
  	char *fptr;
	std::list<char*>::iterator list_iterator;

	// otworz plik konfiguracyjny
	file=fopen(file_name_and_path, "r");
	if ( file==NULL ) 
	{
		printf ("UI fill_section_list Wrong file_name: %s\n", file_name_and_path);
		exit(0);
	}
	 
	// sczytaj nazwy wszytkich sekcji na liste dynamiczna
	fptr = fgets(line,255,file);  // get input line
	
	// dopoki nie osiagnieto konca pliku
	
	while (!feof(file)	)
	{
		// jesli znaleziono nowa sekcje
		if (( fptr!=NULL )&&( line[0]=='[' ))
		{
			char current_section[50];
		    strncpy(current_section, line, strlen(line)-1); 
		    current_section[strlen(line)-1]='\0';

			//		printf("outside: %s\n", current_section);
		
			// checking if section is already considered
			for (list_iterator = ui_state.section_list.begin(); list_iterator != ui_state.section_list.end(); list_iterator++)
			{
				if (strcmp(current_section, *list_iterator) == 0 ) break;
			}

			// if the section does not exists
			if (list_iterator == ui_state.section_list.end()) 
			{
			//		printf("inside: %s\n", current_section);
				char * tmp;
				tmp = new char[50];
				strcpy(tmp, current_section);
				ui_state.section_list.push_back(tmp);
			} 
		
		} // end 	if (( fptr!=NULL )&&( line[0]=='[' ))
		
		// odczytaj nowa lnie
		fptr=fgets(line,255,file);  // get input line
	} // end while (!feof(file)	)

	// zamknij plik
	fclose(file);

	return 1;
}



// fills node list
int fill_node_list()
{
//	printf("fill_node_list\n");
	std::list<char*>::iterator node_list_iterator;
	
    DIR* dirp;
    struct dirent* direntp;


	// fill all network nodes list

    dirp = opendir( "/net" );
    if( dirp != NULL ) {
        for(;;) {
            direntp = readdir( dirp );
            if( direntp == NULL ) break;

		char* tmp;
		tmp = new char[30];
		strcpy(tmp, direntp->d_name);
          ui_state.all_node_list.push_back(tmp);
        }

        closedir( dirp );

    }
	
	
	for (std::list<char*>::iterator section_list_iterator = ui_state.section_list.begin(); section_list_iterator != ui_state.section_list.end(); section_list_iterator++)
	{	
		
		if (config->exists("node_name", *section_list_iterator))
		{
			char* tmp = config->return_string_value("node_name", *section_list_iterator);
	//		printf("bbb: %s\n", config->return_string_value("node_name", *section_list_iterator));
			
			for (node_list_iterator = ui_state.config_node_list.begin(); node_list_iterator != ui_state.config_node_list.end(); node_list_iterator++)
			{
				if (strcmp(tmp, *node_list_iterator) == 0 ) 
				{
		//			printf("bbb\n");
					break;
				}
			}
			
			// if the node does not exists
			if (node_list_iterator == ui_state.config_node_list.end()) 
			{
		//		printf("aaa\n");
				ui_state.config_node_list.push_back(tmp);
			} else {
				delete tmp;
			}
			
		}

	}

return 1;
}



// fills program_node list
int fill_program_node_list()
{
//	printf("fill_program_node_list\n");
	
	for (std::list<char*>::iterator section_list_iterator = ui_state.section_list.begin(); section_list_iterator != ui_state.section_list.end(); section_list_iterator++)
	{	
		
		if ((config->exists("program_name", *section_list_iterator) && config->exists("node_name", *section_list_iterator)))
		{
			//	char* tmp_p = config->return_string_value("program_name", *section_list_iterator);
			//	char* tmp_n = config->return_string_value("node_name", *section_list_iterator);
			
			program_node_def* tmp_s;
			tmp_s = new program_node_def;
			
			tmp_s->program_name = config->return_string_value("program_name", *section_list_iterator);
			tmp_s->node_name = config->return_string_value("node_name", *section_list_iterator);

			ui_state.program_node_list.push_back(*tmp_s);
			
		}
	}

	return 1;
}



// odczytuje nazwe domyslengo pliku konfiguracyjnego, w razie braku ustawia common.ini
int
get_default_configuration_file_name() {

	FILE *fp;
	char *tmp_buf, *tmp_buf1;

	fp = fopen("../configs/default_file.cfg","r");
	if (fp != NULL)
		{
		tmp_buf = new char[255];
		fgets(tmp_buf, 255,fp); // Uwaga na zwracanego NULLa
		tmp_buf1=strtok(tmp_buf,"=\n\r");   // get first token
		strcpy(ui_state.config_file, tmp_buf1);
		
		 strcpy(ui_state.config_file_relativepath, "../configs/");
		 strcat(ui_state.config_file_relativepath, ui_state.config_file);
			
		delete tmp_buf;
		fclose(fp);
		return 1;			
				
	} else {
		// jesli plik z domyslna konfiguracja (default_file.cfg) nie istnieje to utworz go i wpisz do niego common.ini 
		printf ("Utworzono plik default_file.cfg z konfiguracja common.ini\n");
		fp = fopen("../configs/default_file.cfg","w");
		fclose(fp);
		
		 strcpy(ui_state.config_file, "common.ini");
		 strcpy(ui_state.config_file_relativepath, "../configs/");
		 strcat(ui_state.config_file_relativepath, ui_state.config_file);

		  ofstream outfile("../configs/default_file.cfg", ios::out);
	      if (!outfile) {
			  std::cerr << "Cannot open file: default_file.cfg\n";
			perror("because of");	    
			}
          else
			outfile << ui_state.config_file;

	      outfile.close();
		
		return 2;
	}

}




// zapisuje nazwe domyslengo pliku konfiguracyjnego
int
set_default_configuration_file_name() {


	 strcpy(ui_state.config_file_relativepath, "../configs/");
	 strcat(ui_state.config_file_relativepath, ui_state.config_file);

	  ofstream outfile("../configs/default_file.cfg", ios::out);
      if (!outfile) {
		  std::cerr << "Cannot open file: default_file.cfg\n";	
		perror("because of");	    
		}
         else
		outfile << ui_state.config_file;

      outfile.close();
	
	return 1;
}




// ustala stan wszytkich EDP
int check_edps_state_and_modify_mp_state ()
{

	// wyznaczenie stanu wszytkich EDP abstahujac od MP

	// jesli wszytkie sa nieaktywne
	if (
	(!(ui_state.irp6_postument.is_active))
	&&
	 (!(ui_state.irp6_on_track.is_active))
	&&
	 (!(ui_state.conveyor.is_active))
	&&
	 (!(ui_state.speaker.is_active))
 	&&
 	(!(ui_state.irp6_mechatronika.is_active))
	) 
	{
		ui_state.all_edps = UI_ALL_EDPS_NONE_EDP_ACTIVATED;
		
	// jesli wszystkie sa zsynchrnizowane
	} else	 
	 if (
	(((ui_state.irp6_postument.is_active)&&(ui_state.irp6_postument.edp.is_synchronised))||(!(ui_state.irp6_postument.is_active)))
	&&
	 (((ui_state.irp6_on_track.is_active)&&(ui_state.irp6_on_track.edp.is_synchronised))||(!(ui_state.irp6_on_track.is_active)))
	&&
	 (((ui_state.conveyor.is_active)&&(ui_state.conveyor.edp.is_synchronised))||(!(ui_state.conveyor.is_active)))
	&&
	 (((ui_state.speaker.is_active)&&(ui_state.speaker.edp.is_synchronised))||(!(ui_state.speaker.is_active)))
 	&&
 	(((ui_state.irp6_mechatronika.is_active)&&(ui_state.irp6_mechatronika.edp.is_synchronised))||(!(ui_state.irp6_mechatronika.is_active)))
	) 
	{
		ui_state.all_edps = UI_ALL_EDPS_LOADED_AND_SYNCHRONISED; 
	
	// jesli wszystkie sa zaladowane
	} else	 if (
	(((ui_state.irp6_postument.is_active)&&(ui_state.irp6_postument.edp.state>0))||(!(ui_state.irp6_postument.is_active)))
	&&
	 (((ui_state.irp6_on_track.is_active)&&(ui_state.irp6_on_track.edp.state>0))||(!(ui_state.irp6_on_track.is_active)))
	&&
	 (((ui_state.conveyor.is_active)&&(ui_state.conveyor.edp.state>0))||(!(ui_state.conveyor.is_active)))
	&&
	 (((ui_state.speaker.is_active)&&(ui_state.speaker.edp.state>0))||(!(ui_state.speaker.is_active)))
	&& 
 	(((ui_state.irp6_mechatronika.is_active)&&(ui_state.irp6_mechatronika.edp.state>0))||(!(ui_state.irp6_mechatronika.is_active)))
	) 
	{
		ui_state.all_edps = UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED;
		
	// jesli chociaz jeden jest zaladowany
	} else	 if (
	((ui_state.irp6_postument.is_active)&&(ui_state.irp6_postument.edp.state>0))
	||
	 ((ui_state.irp6_on_track.is_active)&&(ui_state.irp6_on_track.edp.state>0))
	||
	 ((ui_state.conveyor.is_active)&&(ui_state.conveyor.edp.state>0))
	||
	 ((ui_state.speaker.is_active)&&(ui_state.speaker.edp.state>0))
	|| 
 	((ui_state.irp6_mechatronika.is_active)&&(ui_state.irp6_mechatronika.edp.state>0))
	) 
		
	{
		ui_state.all_edps = UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED;
		
	// jesli zaden nie jest zaladowany
	} else {
		ui_state.all_edps = UI_ALL_EDPS_NONE_EDP_LOADED;
		
	}

	// modyfikacja stanu MP przez stan wysztkich EDP

	switch (ui_state.all_edps)
	{
		case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
		case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
			if ((ui_state.mp.state == UI_MP_NOT_PERMITED_TO_RUN) && (ui_state.is_mp_and_ecps_active))
			{
				ui_state.mp.state = UI_MP_PERMITED_TO_RUN; // pozwol na uruchomienie mp
			}	
		break;

		case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
		case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
		case UI_ALL_EDPS_NONE_EDP_LOADED:
			if (ui_state.mp.state == UI_MP_PERMITED_TO_RUN)
			{
				ui_state.mp.state = UI_MP_NOT_PERMITED_TO_RUN; // nie pozwol na uruchomienie mp
			}
		break;
		default:
		break;
	}


}


int
pulse_reader_execute( int coid, int pulse_code, int pulse_value)

{
	
	if (MsgSendPulse (coid, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value)==-1) 
	{
		perror("Blad w wysylaniu pulsu do redera");
	}
	
	return 1;
}



	
// sprawdza czy sa postawione gns's i ew. stawia je
// uwaga serwer powinien byc wczesniej postawiony (dokladnie jeden w sieci)

int
check_gns()
	{
	DIR* dirp;
   	unsigned short number_of_gns_serwers = 0;
	 
 	 if (access("/etc/system/config/useqnet", R_OK)) 
	{
	 	printf ("UI: There is no /etc/system/config/useqnet file; the qnet will not work properly.\n");
		exit(0);
	 }	
   	
   	// poszukiwanie serwerow gns
    	for (std::list<char*>::iterator node_list_iterator = ui_state.all_node_list.begin(); node_list_iterator != ui_state.all_node_list.end(); node_list_iterator++)
	{
   		char opendir_path[100];
						
		strcpy(opendir_path, "/net/");
		strcat(opendir_path, *node_list_iterator);
		strcat(opendir_path, "/proc/mount/dev/name/gns_server");
		
 	 	// sprawdzenie czy dziala serwer gns
 	   	 if ((dirp = opendir(opendir_path))!=NULL) 
		{
			number_of_gns_serwers++;
		 	closedir( dirp );
		 }

	}
   	
   	// there is more than one gns server in the QNX network
   	if (number_of_gns_serwers > 1)
   	{
   		printf ("UI: There is more than one gns server in the QNX network; the qnet will not work properly.\n");
   		// printing of gns server nodes
   	    	for (std::list<char*>::iterator node_list_iterator = ui_state.all_node_list.begin(); node_list_iterator != ui_state.all_node_list.end(); node_list_iterator++)
		{
	   		char opendir_path[100];
							
			strcpy(opendir_path, "/net/");
			strcat(opendir_path, *node_list_iterator);
			strcat(opendir_path, "/proc/mount/dev/name/gns_server");
			
	 	 	// sprawdzenie czy dziala serwer gns
	 	   	 if ((dirp = opendir(opendir_path))!=NULL) 
			{
			 	closedir( dirp );
				printf ("There gns server is set on %s node\n", *node_list_iterator);
			 }
		}
   		
		exit(0);
   	}
   	// gns server was not found in the QNX network
   	else if (!number_of_gns_serwers)
   	{
   		printf("UI: gns server was not found in the QNX network, it will be automatically run on local node\n");
		
		// ew. zabicie klienta gns
		if ((dirp = opendir( "/proc/mount/dev/name" )) != NULL) 
		{
			closedir( dirp );
			system("slay gns");
		 } 
		
		// uruchomienie serwera
		system("gns -s");
   	}
   	
   	
    	// sprawdzanie lokalne
  	 if ((dirp = opendir( "/proc/mount/dev/name" )) == NULL) 
	{
		system("gns");
	 } else 
	{
		 closedir( dirp );
	}

	// sprawdzenie czy wezly w konfiuracji sa uruchomione i ew. uruchomienie na nich brakujacych klientow gns

 	for (std::list<char*>::iterator node_list_iterator = ui_state.config_node_list.begin(); node_list_iterator != ui_state.config_node_list.end(); node_list_iterator++)
	{

		char opendir_path[100];
						
		strcpy(opendir_path, "/net/");
		strcat(opendir_path, *node_list_iterator);
	
		// sprawdzenie czy istnieje wezel
		if ((dirp = opendir (opendir_path)) != NULL) 
		{
			closedir( dirp );
			strcat(opendir_path, "/proc/mount/dev/name");

	 	 	// sprawdzenie czy dziala gns
	 	   	 if ((dirp = opendir(opendir_path))==NULL) 
			{
				char system_command[100];
															
				strcpy(system_command, "on -f ");
				strcat(system_command, *node_list_iterator);
				strcat(system_command, " gns");
			
				system(system_command);
			 }  else 
			{
				 closedir( dirp );
			}
	
		} else 
		{
			printf("check_gns - Nie wykryto wezla: %s, ktory wystepuje w pliku konfiguracyjnym\n", *node_list_iterator);
		}
	}
	

	return 0;

	};



