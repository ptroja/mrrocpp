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
#include <process.h>
#include <math.h>


#include "lib/srlib.h"
#include "ui/ui_const.h"
// #include "ui/ui.h"
// Konfigurator.
// #include "lib/configurator.h"
#include "ui/ui_ecp.h"



/* Local headers */
#include "proto.h"



extern ui_msg_def ui_msg;
extern ui_ecp_buffer* ui_ecp_obj;

extern ui_state_def ui_state;
extern configurator* config;

extern ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;


// zamykanie okna odtwarzania dzwiekow dla robota speaker

//jk
int speaker_preset_sound0()
{
	char local_text[MAX_TEXT];
	char local_prosody[MAX_PROSODY];
	try
	{
		if (ui_state.speaker.edp.pid!=-1)
		{
			strcpy(local_text, ui_state.speaker.edp.preset_sound_0);
			strcpy(local_prosody, "neutral");
			speaker_check_state();
			ui_robot.speaker->send_command(local_text, local_prosody);
			speaker_check_state();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int speaker_preset_sound1()
{
	char local_text[MAX_TEXT];
	char local_prosody[MAX_PROSODY];
	try
	{
		if (ui_state.speaker.edp.pid!=-1)
		{
			strcpy(local_text, ui_state.speaker.edp.preset_sound_1);
			strcpy(local_prosody, "neutral");
			speaker_check_state();
			ui_robot.speaker->send_command(local_text, local_prosody);
			speaker_check_state();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int speaker_preset_sound2()
{
	char local_text[MAX_TEXT];
	char local_prosody[MAX_PROSODY];
	try
	{
		if (ui_state.speaker.edp.pid!=-1)
		{
			strcpy(local_text, ui_state.speaker.edp.preset_sound_2);
			strcpy(local_prosody, "neutral");
			speaker_check_state();
			ui_robot.speaker->send_command(local_text, local_prosody);
			speaker_check_state();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int speaker_play_exec()
{
	char local_text[MAX_TEXT];
	char local_prosody[MAX_PROSODY];
	const char* ref_local_text = "test";
	const char* ref_local_prosody = "neutral";

	try
	{
		if (ui_state.speaker.edp.pid!=-1)
		{
			strcpy(local_text, ref_local_text);
			strcpy(local_prosody, ref_local_prosody);
			speaker_check_state();
			ui_robot.speaker->send_command(local_text, local_prosody);
			speaker_check_state();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int speaker_check_state()
{
	try
	{
		if (ui_state.speaker.edp.pid!=-1)
		{
			ui_robot.speaker->read_state(&(ui_robot.speaker->speaking_state));

			double* v = new double[1];
			if (ui_robot.speaker->speaking_state)
			{ // odtwarzanie w toku
				v[0] = 0;
			}
			else
			{
				v[0] = 1;
			}
			replySend(new Message('E','A','B',1,v,NULL));
		}
	}
	CATCH_SECTION_UI

	return NULL;
}


int EDP_speaker_create()
{
	set_ui_state_notification(UI_N_PROCESS_CREATION);
char tmp_string[100];
	char tmp2_string[100];

	try
	{
		if (ui_state.speaker.edp.state == 0)
		{
			strcpy(tmp_string, "/dev/name/global/");
			strcat(tmp_string, ui_state.speaker.edp.hardware_busy_attach_point);

			strcpy(tmp2_string, "/dev/name/global/");
			strcat(tmp2_string, ui_state.speaker.edp.network_resourceman_attach_point);

			// sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow
			if( (!(ui_state.speaker.edp.test_mode)) && (access(tmp_string, R_OK)== 0 )
				|| (access(tmp2_string, R_OK)== 0 )
			)
			{
				ui_msg.ui->message("edp_speaker already exists");

			} else {

				ui_state.speaker.edp.node_nr = config->return_node_number(ui_state.speaker.edp.node_name);

				ui_robot.speaker = new ui_speaker_robot(&ui_state.speaker.edp, *config, ui_msg.all_ecp);
				ui_state.speaker.edp.pid = ui_robot.speaker->get_EDP_pid();
				replySend(new Message('E','B','A',0,NULL,NULL));
				if (ui_state.speaker.edp.pid<0)
				{
					fprintf( stderr, "EDP spawn failed: %s\n", strerror( errno ));
					delete ui_robot.speaker;
				}
				else
				{  // jesli spawn sie powiodl
					ui_state.speaker.edp.state=1;
					ui_state.speaker.edp.is_synchronised=true;
				}
			}
		}
	}
	CATCH_SECTION_UI
manage_interface();
	return 0;
}

int EDP_speaker_slay()
{
	if (ui_state.speaker.edp.state>0)
	{
		name_close(ui_state.speaker.edp.reader_fd);
		delete ui_robot.speaker;

		ui_state.speaker.edp.state = 0; // edp wylaczone
		ui_state.speaker.edp.is_synchronised = false;

		ui_state.speaker.edp.pid = -1;
		ui_state.speaker.edp.reader_fd = -1;
	}
	replySend(new Message('E','C','A',0,NULL,NULL));
	manage_interface();
	return 0;
}

int pulse_reader_speaker_start()
{
	pulse_reader_speaker_start_exec_pulse();
manage_interface();
	return 0;
}

int pulse_reader_speaker_stop()
{
	pulse_reader_speaker_stop_exec_pulse();
manage_interface();
	return 0;
}

int pulse_reader_speaker_trigger()
{
	pulse_reader_speaker_trigger_exec_pulse();
manage_interface();
	return 0;
}

int pulse_ecp_speaker()
{
	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	if (ui_state.speaker.edp.is_synchronised>0)
	{
		if (ui_state.speaker.ecp.trigger_fd <0)
		{
			 short tmp = 0;
		 	ualarm( (useconds_t)( SIGALRM_TIMEOUT), 0);
			while( (ui_state.speaker.ecp.trigger_fd = name_open(ui_state.speaker.ecp.network_trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
			{
				if (errno == EINTR) break;
				if((tmp++)<20)
					delay(50);
				else{
				   perror("blad odwolania do ECP_TRIGGER\n");
				};
			}
			// odwolanie alarmu
			ualarm( (useconds_t)( 0), 0);
		}

		if (ui_state.speaker.ecp.trigger_fd >=0) {
			if (MsgSendPulse (ui_state.speaker.ecp.trigger_fd, sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {

				fprintf( stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",  strerror( errno ) );
				delay(1000);
			}
		}
		else
		{
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}
manage_interface();
	return 0;
}
//~jk




bool pulse_reader_speaker_start_exec_pulse ()
{

	if (ui_state.speaker.edp.state == 1)
	{
		pulse_reader_execute( ui_state.speaker.edp.reader_fd, READER_START, 0);
		ui_state.speaker.edp.state = 2;
		return true;
	}

	return false;
}


bool pulse_reader_speaker_stop_exec_pulse ()
{

	if (ui_state.speaker.edp.state == 2)
	{
		pulse_reader_execute( ui_state.speaker.edp.reader_fd, READER_STOP, 0);
		ui_state.speaker.edp.state = 1;
		return true;
	}

	return false;
}


bool pulse_reader_speaker_trigger_exec_pulse ()
{

	if (ui_state.speaker.edp.state == 2)
	{
		pulse_reader_execute( ui_state.speaker.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}

	return false;
}



int
reload_speaker_configuration ()
{


	// jesli speaker ma byc aktywny
	if ((ui_state.speaker.is_active = config->return_int_value("is_speaker_active")) == 1)
	{

		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active)
		{
			delete [] ui_state.speaker.ecp.network_trigger_attach_point;
			ui_state.speaker.ecp.network_trigger_attach_point =config->return_attach_point_name
				(configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.speaker.ecp.section_name);

	 		ui_state.speaker.ecp.pid = -1;
	 		ui_state.speaker.ecp.trigger_fd = -1;
	 	}

		switch (ui_state.speaker.edp.state)
		{
			case -1:
			case 0:

				ui_state.speaker.edp.pid = -1;
				ui_state.speaker.edp.reader_fd = -1;
				ui_state.speaker.edp.state = 0;



				if (config->exists("test_mode", ui_state.speaker.edp.section_name))
					ui_state.speaker.edp.test_mode = config->return_int_value("test_mode", ui_state.speaker.edp.section_name);
				else
					ui_state.speaker.edp.test_mode = 0;

				delete [] ui_state.speaker.edp.hardware_busy_attach_point;
				ui_state.speaker.edp.hardware_busy_attach_point = config->return_string_value
					("hardware_busy_attach_point", ui_state.speaker.edp.section_name);



				delete [] ui_state.speaker.edp.network_resourceman_attach_point;
				ui_state.speaker.edp.network_resourceman_attach_point = config->return_attach_point_name
					(configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.speaker.edp.section_name);

				delete [] ui_state.speaker.edp.network_reader_attach_point;
				ui_state.speaker.edp.network_reader_attach_point = config->return_attach_point_name
					(configurator::CONFIG_SERVER, "reader_attach_point", ui_state.speaker.edp.section_name);

				delete [] ui_state.speaker.edp.node_name;
				ui_state.speaker.edp.node_name = config->return_string_value ("node_name", ui_state.speaker.edp.section_name);

				delete [] ui_state.speaker.edp.preset_sound_0;
				delete [] ui_state.speaker.edp.preset_sound_1;
				delete [] ui_state.speaker.edp.preset_sound_2;
				ui_state.speaker.edp.preset_sound_0 = config->return_string_value("preset_sound_0", ui_state.speaker.edp.section_name);
				ui_state.speaker.edp.preset_sound_1 = config->return_string_value("preset_sound_1", ui_state.speaker.edp.section_name);
				ui_state.speaker.edp.preset_sound_2 = config->return_string_value("preset_sound_2", ui_state.speaker.edp.section_name);

			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}

	} else // jesli  conveyor ma byc nieaktywny
	{

		switch (ui_state.speaker.edp.state)
		{
			case -1:
			case 0:
				ui_state.speaker.edp.state=-1;
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	} // end speaker

	return 1;
}

int
manage_interface_speaker ()
{
	return 1;
}
