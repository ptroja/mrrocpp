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
extern lib::configurator* config;

extern ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;

double conveyor_current_pos[CONVEYOR_NUM_OF_SERVOS];// pozycja biezaca
double conveyor_desired_pos[CONVEYOR_NUM_OF_SERVOS]; // pozycja zadana


// zamykanie okien ruchow recznych dla robota irp6_on_track



// zamykanie okien ruchow recznych dla robota conveyor

//jk

int
process_control_window_conveyor_section_init (bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{
double* tmp = new double[2];
	if (ui_state.conveyor.edp.state<=0) {// edp wylaczone
		tmp[0] = 2;
		tmp[1] = 0;
		replySend(new Message('A','H','A',2,tmp,NULL));
	} else {
		if (ui_state.conveyor.edp.state==1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start=true;
			tmp[0] = 2;
			tmp[1] = 4;
			replySend(new Message('A','H','A',2,tmp,NULL));
		} else if (ui_state.conveyor.edp.state==2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop=true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger=true;
			tmp[0] = 2;
			tmp[1] = 3;
			replySend(new Message('A','H','A',2,tmp,NULL));
		}
	}
	return 1;
}

int
pulse_reader_conv_trigger()
{
	pulse_reader_conv_trigger_exec_pulse();

	return 0;
}


int
pulse_ecp_conveyor()
{
	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	if (ui_state.conveyor.edp.is_synchronised>0)
	{ // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui_state.conveyor.ecp.trigger_fd < 0)
		{

			 short tmp = 0;
		 	// kilka sekund  (~1) na otworzenie urzadzenia
		 	// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem
		 	ualarm( (useconds_t)( SIGALRM_TIMEOUT), 0);
			while( (ui_state.conveyor.ecp.trigger_fd = name_open(ui_state.conveyor.ecp.network_trigger_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0)
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

		if (ui_state.conveyor.ecp.trigger_fd >= 0) {
			if (MsgSendPulse (ui_state.conveyor.ecp.trigger_fd, sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {

				fprintf( stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",  strerror( errno ) );
				delay(1000);
			}
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return 0;
}

int
manage_interface_conveyor ()
{
	switch (ui_state.conveyor.edp.state)
	{
		case -1:
			replySend(new Message('D','E','A',0,NULL,NULL));
		break;
		case 0:
			replySend(new Message('D','E','B',0,NULL,NULL));
		break;
		case 1:
		case 2:
			replySend(new Message('D','E','C',0,NULL,NULL));

			// jesli robot jest zsynchronizowany
			if (	ui_state.conveyor.edp.is_synchronised)
			{
				replySend(new Message('D','E','D',0,NULL,NULL));
				switch (ui_state.mp.state)
				{
					case UI_MP_NOT_PERMITED_TO_RUN:
					case UI_MP_PERMITED_TO_RUN:
						replySend(new Message('D','E','E',0,NULL,NULL));
					break;
					case UI_MP_WAITING_FOR_START_PULSE:
						replySend(new Message('D','E','F',0,NULL,NULL));
					break;
					case UI_MP_TASK_RUNNING:
					case UI_MP_TASK_PAUSED:
							replySend(new Message('D','E','G',0,NULL,NULL));
					break;
					default:
					break;
				}
			} else		// jesli robot jest niezsynchronizowany
			{
				replySend(new Message('D','E','H',0,NULL,NULL));
				replySend(new Message('A','F','A',0,NULL,NULL));
			}
		break;
		default:
		break;
	}

	return 1;
}



int conveyor_read_servo_algorithm()
{
	lib::BYTE servo_alg_no[1];
	lib::BYTE servo_par_no[1];
	ui_robot.conveyor->get_servo_algorithm(servo_alg_no, servo_par_no);
	double* v = new double[2];
	v[0] = servo_alg_no[0];
	v[1] = servo_par_no[0];
	replySend(new Message('D','B','A',2,v,NULL));
}
int conveyor_read_joints()
{
	double* v = new double[2];
	double* vv = new double[1];
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			if ( ui_state.conveyor.edp.is_synchronised )
			{
				if (!(ui_robot.conveyor->read_joints(vv))) printf("Blad w read motors\n");
				v[1] = *vv;
				if (!(ui_robot.conveyor->read_motors(vv))) printf("Blad w read motors\n");
				v[0] = *vv;
				replySend(new Message('D','A','A',2,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
}
int conveyor_read_motors()
{
	double* v = new double[2];
	double* vv = new double[1];
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{
			if ( ui_state.conveyor.edp.is_synchronised )
			{
				if (!(ui_robot.conveyor->read_joints(vv))) printf("Blad w read motors\n");
				v[1] = *vv;
				if (!(ui_robot.conveyor->read_motors(vv))) printf("Blad w read motors\n");
				v[0] = *vv;
				replySend(new Message('D','A','A',2,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
}

int conveyor_moves_move_motors(double* v)
{
	double conveyor_desired_pos_motors[6];
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{

				if (ui_state.conveyor.edp.is_synchronised)
				{
					conveyor_desired_pos_motors[0] = v[0];
				}
				else
				{
					conveyor_desired_pos_motors[0]=0.0;
				}
				ui_robot.conveyor->move_motors(conveyor_desired_pos_motors);
				conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int conveyor_moves_move_joints(double* v)
{
	double conveyor_desired_pos_int[6] ;
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{

				if (ui_state.conveyor.edp.is_synchronised)
				{
					conveyor_desired_pos_int[0] = v[0];
				}
				else
				{
					conveyor_desired_pos_int[0]=0.0;
				}
				ui_robot.conveyor->move_joints(conveyor_desired_pos_int);
				conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int conv_servo_algorithm_set(double* v)
{
	lib::BYTE servo_alg_no_output[CONVEYOR_NUM_OF_SERVOS];
	lib::BYTE servo_par_no_output[CONVEYOR_NUM_OF_SERVOS];

	try
	{
		if ( ui_state.conveyor.edp.is_synchronised )
		{
			for(int i=0; i<CONVEYOR_NUM_OF_SERVOS; i++)
			{
				servo_alg_no_output[i] = (char)v[2*i];
				servo_par_no_output[i] = (char)v[2*i+1];
			}
			ui_robot.conveyor->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);
			conveyor_read_servo_algorithm();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_synchro_position()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{

			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = 0.0;
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_position0()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{

			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[0][i];
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_position1()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{

			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[1][i];
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int conveyor_move_to_position2()
{
	try
	{
		if (ui_state.conveyor.edp.pid!=-1)
		{

			for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
			{
				conveyor_desired_pos[i] = ui_state.conveyor.edp.preset_position[2][i];
		    }
			ui_robot.conveyor->move_motors(conveyor_desired_pos);
			conveyor_read_joints();
			conveyor_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int EDP_conveyor_synchronise()
{
	set_ui_state_notification(UI_N_SYNCHRONISATION);
	try
	{
		if ((ui_state.conveyor.edp.state > 0) && (ui_state.conveyor.edp.is_synchronised == false))
		{
			ui_robot.conveyor->synchronise();
			ui_state.conveyor.edp.is_synchronised = ui_robot.conveyor->is_synchronised();
		}
		if ((ui_state.conveyor.edp.state > 0) && (ui_state.conveyor.edp.is_synchronised == true)) replySend(new Message('D','E','A',0,NULL,NULL));
	}
	CATCH_SECTION_UI

	manage_interface();
	return 0;
}

int EDP_conveyor_create()
{
	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try {
		if (ui_state.conveyor.edp.state == 0)
		{
			ui_state.conveyor.edp.state = 0;
			ui_state.conveyor.edp.is_synchronised = false;

			std::string busy_attach_point("/dev/name/global/");
			busy_attach_point += ui_state.conveyor.edp.hardware_busy_attach_point;

			std::string resourceman_attach_point("/dev/name/global/");
			resourceman_attach_point += ui_state.conveyor.edp.network_resourceman_attach_point;

			if((!(ui_state.conveyor.edp.test_mode)) && ( access(busy_attach_point.c_str(), R_OK)== 0  )
				|| (access(resourceman_attach_point.c_str(), R_OK)== 0 )
			)
			{
				ui_msg.ui->message("edp_conveyor already exists");
			} else {
				ui_state.conveyor.edp.node_nr = config->return_node_number(ui_state.conveyor.edp.node_name);
				ui_state.conveyor.edp.state = 1;
				ui_robot.conveyor = new ui_conveyor_robot(*config, *ui_msg.all_ecp);
				ui_state.conveyor.edp.pid = ui_robot.conveyor->get_EDP_pid();

				if (ui_state.conveyor.edp.pid<0)
				{
					ui_state.conveyor.edp.state = 0;
					fprintf( stderr, "EDP spawn failed: %s\n", strerror( errno ));
					delete ui_robot.conveyor;
				} else {  // jesli spawn sie powiodl

					short tmp = 0;
				 	// kilka sekund  (~1) na otworzenie urzadzenia
					while((ui_state.conveyor.edp.reader_fd = name_open(ui_state.conveyor.edp.network_reader_attach_point.c_str(),
						NAME_FLAG_ATTACH_GLOBAL))  < 0)
						if((tmp++)<20)
							delay(50);
						else{
						   perror("blad odwolania do READER_C\n");
		   				   break;
						};

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;
					ui_robot.conveyor->get_controller_state(&robot_controller_initial_state_tmp);

					//ui_state.conveyor.edp.state = 1; // edp wlaczone reader czeka na start
					replySend(new Message('D','C','A',0,NULL,NULL));
					ui_state.conveyor.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}
	}
	CATCH_SECTION_UI
manage_interface();
	return 0;
}

int EDP_conveyor_slay()
{
	if (ui_state.conveyor.edp.state>0)
	 { // jesli istnieje EDP
		name_close(ui_state.conveyor.edp.reader_fd);
		delete ui_robot.conveyor;
		ui_state.conveyor.edp.state = 0; // edp wylaczone
		ui_state.conveyor.edp.is_synchronised = false;
		ui_state.conveyor.edp.pid = -1;
		ui_state.conveyor.edp.reader_fd = -1;
	}

	replySend(new Message('D','D','A',0,NULL,NULL));
	manage_interface();
	return 0;
}

int
pulse_reader_conv_start()
{
	pulse_reader_conv_start_exec_pulse();

	return 0;
}

int
pulse_reader_conv_stop()
{
	pulse_reader_conv_stop_exec_pulse();

	return 0;
}

//~jk


bool pulse_reader_conv_start_exec_pulse ()
{

	if (ui_state.conveyor.edp.state == 1)
	{
		pulse_reader_execute( ui_state.conveyor.edp.reader_fd, READER_START, 0);
		ui_state.conveyor.edp.state = 2;
		return true;
	}

	return false;
}




bool pulse_reader_conv_stop_exec_pulse ()
{

	if (ui_state.conveyor.edp.state == 2)
	{
		pulse_reader_execute( ui_state.conveyor.edp.reader_fd, READER_STOP, 0);
		ui_state.conveyor.edp.state = 1;
		return true;
	}

	return false;
}



bool pulse_reader_conv_trigger_exec_pulse ()
{

	if (ui_state.conveyor.edp.state == 2)
	{
		pulse_reader_execute( ui_state.conveyor.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}

	return false;
}




int
reload_conveyor_configuration ()
{


	// jesli conveyor ma byc aktywny
	if ((ui_state.conveyor.is_active = config->return_int_value("is_conveyor_active")) == 1)
	{

		//ui_state.is_any_edp_active = true;

		if (ui_state.is_mp_and_ecps_active)
		{
			ui_state.conveyor.ecp.network_trigger_attach_point =config->return_attach_point_name
				(lib::configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.conveyor.ecp.section_name);

	 		ui_state.conveyor.ecp.pid = -1;
	 		ui_state.conveyor.ecp.trigger_fd = -1;
	 	}

		switch (ui_state.conveyor.edp.state)
		{
			case -1:
			case 0:

				ui_state.conveyor.edp.pid = -1;
				ui_state.conveyor.edp.reader_fd = -1;
				ui_state.conveyor.edp.state = 0;

				if (config->exists("preset_position_0", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.preset_position[0][0] = config->return_double_value ("preset_position_0", ui_state.conveyor.edp.section_name);
				if (config->exists("preset_position_1", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.preset_position[1][0] = config->return_double_value ("preset_position_1", ui_state.conveyor.edp.section_name);
				if (config->exists("preset_position_2", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.preset_position[2][0] = config->return_double_value ("preset_position_2", ui_state.conveyor.edp.section_name);

				if (config->exists("test_mode", ui_state.conveyor.edp.section_name))
					ui_state.conveyor.edp.test_mode = config->return_int_value("test_mode", ui_state.conveyor.edp.section_name);
				else
					ui_state.conveyor.edp.test_mode = 0;

				ui_state.conveyor.edp.hardware_busy_attach_point = config->return_string_value
					("hardware_busy_attach_point", ui_state.conveyor.edp.section_name);



				ui_state.conveyor.edp.network_resourceman_attach_point = config->return_attach_point_name
					(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.conveyor.edp.section_name);

				ui_state.conveyor.edp.network_reader_attach_point = config->return_attach_point_name
					(lib::configurator::CONFIG_SERVER, "reader_attach_point", ui_state.conveyor.edp.section_name);

				ui_state.conveyor.edp.node_name = config->return_string_value ("node_name", ui_state.conveyor.edp.section_name);

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

		switch (ui_state.conveyor.edp.state)
		{
			case -1:
			case 0:
				ui_state.conveyor.edp.state=-1;
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	} // end conveyor

	return 1;
}


