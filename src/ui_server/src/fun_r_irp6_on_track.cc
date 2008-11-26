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
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "proto.h"


extern ui_msg_def ui_msg;
extern ui_ecp_buffer* ui_ecp_obj;

extern ui_state_def ui_state;
extern configurator* config;

extern ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;

double irp6ot_current_pos[8]; // pozycja biezaca
double irp6ot_desired_pos[8]; // pozycja zadana

// zamykanie okien ruchow recznych dla robota irp6_on_track


//jk
int
process_control_window_irp6ot_section_init (bool &wlacz_PtButton_wnd_processes_control_all_reader_start,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_stop,
	bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{
double* tmp = new double[2];

	if (ui_state.irp6_on_track.edp.state<=0) {// edp wylaczone
		tmp[0] = 0;
		tmp[1] = 0;
		replySend(new Message('A','H','A',2,tmp,NULL));
	} else {
		if (ui_state.irp6_on_track.edp.state==1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start=true;
			tmp[0] = 0;
			tmp[1] = 4;
			replySend(new Message('A','H','A',2,tmp,NULL));
		} else if (ui_state.irp6_on_track.edp.state==2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop=true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger=true;
			tmp[0] = 0;
			tmp[1] = 3;
			replySend(new Message('A','H','A',2,tmp,NULL));
		}
	}
	return 1;
}


int 
manage_interface_irp6ot ()
{

	switch (ui_state.irp6_on_track.edp.state)
	{
	
		case -1:

			replySend(new Message('B','M','A',0,NULL,NULL));
		break;
		case 0:

			replySend(new Message('B','M','B',0,NULL,NULL));
			
		break;
		case 1:
		case 2:

			replySend(new Message('B','M','C',0,NULL,NULL));
			// jesli robot jest zsynchronizowany
			if (	ui_state.irp6_on_track.edp.is_synchronised)
			{
				replySend(new Message('B','M','D',0,NULL,NULL));
				
				switch (ui_state.mp.state)
				{
					case UI_MP_NOT_PERMITED_TO_RUN:
					case UI_MP_PERMITED_TO_RUN:
						replySend(new Message('B','M','E',0,NULL,NULL));
					break;
					case UI_MP_WAITING_FOR_START_PULSE:
						replySend(new Message('B','M','F',0,NULL,NULL));
					break;
					case UI_MP_TASK_RUNNING: 
					case UI_MP_TASK_PAUSED:
							replySend(new Message('B','M','G',0,NULL,NULL));
					break;
					default:
					break;
				}

			} else		// jesli robot jest niezsynchronizowany
			{		
				replySend(new Message('B','M','H',0,NULL,NULL));
				replySend(new Message('A','G','A',0,NULL,NULL));
			}
		break;
		default:
		break;

	}
	
	return 1;
}




int irp6ot_read_kinematic()
{
	BYTE model_no;
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )
			{
				if (!(ui_robot.irp6_on_track->get_kinematic(&model_no))) printf("Blad w read external\n");
				else
				{
					double* v = new double[1];
					v[0] = model_no;
					replySend(new Message('B','A','A',1,v,NULL));
				}
			}
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_read_servo_algorithm()
{
	BYTE servo_alg_no[IRP6_ON_TRACK_NUM_OF_SERVOS];
	BYTE servo_par_no[IRP6_ON_TRACK_NUM_OF_SERVOS];

	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )
			{
				if (!(ui_robot.irp6_on_track->get_servo_algorithm(servo_alg_no, servo_par_no))) printf("Blad w on_track get_servo_algorithm\n");
				else
				{
					double* v = new double[IRP6_ON_TRACK_NUM_OF_SERVOS*2];
					for(int i = 0;i < IRP6_ON_TRACK_NUM_OF_SERVOS;++i)
					{
						v[2*i] = servo_alg_no[i];
						v[2*i+1] = servo_par_no[i];
					}
					replySend(new Message('B','B','A',IRP6_ON_TRACK_NUM_OF_SERVOS*2,v,NULL));
				}
			}
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_read_post_angle_axis()
{
	double* v = new double[8];
	double alfa, kx, ky, kz;

	try
	{	
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
			{
				if (!(ui_robot.irp6_on_track->read_xyz_angle_axis(irp6ot_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read_xyz_angle_axis\n");
		
				v[0] = irp6ot_current_pos[0];
				v[1] = irp6ot_current_pos[1];
				v[2] = irp6ot_current_pos[2];
				v[7] = irp6ot_current_pos[6];
					
				alfa = sqrt(irp6ot_current_pos[3]*irp6ot_current_pos[3]
					+irp6ot_current_pos[4]*irp6ot_current_pos[4]
					+irp6ot_current_pos[5]*irp6ot_current_pos[5]);
				
				kx = irp6ot_current_pos[3]/alfa;
				ky = irp6ot_current_pos[4]/alfa;
				kz = irp6ot_current_pos[5]/alfa;
		
				v[3] = kx;
				v[4] = ky;
				v[5] = kz;
				v[6] = alfa;
				
				replySend(new Message('B','C','A',8,v,NULL));
			} 
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_read_post_euler()
{
	double* v = new double[7];
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )
			{
				if (!(ui_robot.irp6_on_track->read_xyz_euler_zyz(v)))
					printf("Blad w read external\n");
				else replySend(new Message('B','D','A',7,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
	
	return 0;
}

int irp6ot_read_joints()
{
	double* v = new double[IRP6_ON_TRACK_NUM_OF_SERVOS];
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )
			{
				if (!(ui_robot.irp6_on_track->read_joints(v))) printf("Blad w read joints\n");
				else replySend(new Message('B','E','A',IRP6_ON_TRACK_NUM_OF_SERVOS,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_read_motors()
{
	double* v = new double[IRP6_ON_TRACK_NUM_OF_SERVOS];
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )
			{
				if (!(ui_robot.irp6_on_track->read_motors(v))) printf("Blad w read joints\n");
				else replySend(new Message('B','F','A',IRP6_ON_TRACK_NUM_OF_SERVOS,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
	return 0;
}


int irp6ot_read_tool_angle()
{
	double* v = new double[7];
	double alfa, kx, ky, kz;

	try
	{	
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
			{
				if (!(ui_robot.irp6_on_track->read_tool_xyz_angle_axis(v))) // Odczyt polozenia walow silnikow
					printf("Blad w read external\n");

				alfa = sqrt(v[3]*v[3]
					+v[4]*v[4]
					+v[5]*v[5]);
			
				if (alfa==0){
					kx = -1;
					ky = 0;
					kz = 0;
				}
				else{
					kx = v[3]/alfa;
					ky = v[4]/alfa;
					kz = v[5]/alfa;
				}
			
			v[3] = kx;
			v[4] = ky;
			v[5] = kz;
			v[6] = alfa;
			}

			replySend(new Message('B','H','A',7,v,NULL));
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_read_tool_euler()
{
	double* v = new double[6];
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if ( ui_state.irp6_on_track.edp.is_synchronised )
			{
				if (!(ui_robot.irp6_on_track->read_tool_xyz_euler_zyz(v))) printf("Blad w read external\n");
				else replySend(new Message('B','I','A',6,v,NULL));
			}
		}
	}
	CATCH_SECTION_UI
}

int irp6ot_move_to_synchro_position()
{
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{

			if (ui_state.irp6_on_track.edp.is_synchronised)
			{// powrot do pozycji synchronizacji
				for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				{
			          irp6ot_desired_pos[i] = 0.0;
		        }
			}
			ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);
			irp6ot_read_joints();
			irp6ot_read_post_angle_axis();
			irp6ot_read_post_euler();
			irp6ot_read_motors();
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_move_to_position0()
{
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{

		if (ui_state.irp6_on_track.edp.is_synchronised)
		{// powrot do pozycji synchronizacji
			for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				irp6ot_desired_pos[i] = ui_state.irp6_on_track.edp.preset_position[0][i];
			}
		}
		ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);
		irp6ot_read_joints();
		irp6ot_read_post_angle_axis();
		irp6ot_read_post_euler();
		irp6ot_read_motors();
		}
	}
	CATCH_SECTION_UI
	return 0;
}
int irp6ot_move_to_position1()
{
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{

		if (ui_state.irp6_on_track.edp.is_synchronised)
		{// powrot do pozycji synchronizacji
			for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				irp6ot_desired_pos[i] = ui_state.irp6_on_track.edp.preset_position[1][i];
		
			}
		}
		ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);
		irp6ot_read_joints();
		irp6ot_read_post_angle_axis();
		irp6ot_read_post_euler();
		irp6ot_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}
int irp6ot_move_to_position2()
{
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{

		if (ui_state.irp6_on_track.edp.is_synchronised)
		{// powrot do pozycji synchronizacji
			for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				irp6ot_desired_pos[i] = ui_state.irp6_on_track.edp.preset_position[2][i];
			}
		}
		ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);
		irp6ot_read_joints();
		irp6ot_read_post_angle_axis();
		irp6ot_read_post_euler();
		irp6ot_read_motors();
		}
	}
	CATCH_SECTION_UI

	return 0;
}
int irp6ot_inc_motion(double* v)
{
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if (ui_state.irp6_on_track.edp.is_synchronised)
			{
				for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				{
			          irp6ot_desired_pos[i] = v[i];
		        }
			}
		}
		ui_robot.irp6_on_track->move_motors(irp6ot_desired_pos);
		irp6ot_read_joints();
		irp6ot_read_post_angle_axis();
		irp6ot_read_post_euler();
		irp6ot_read_motors();
	}
	CATCH_SECTION_UI

	return 0;
}
int EDP_irp6_on_track_synchronise()
{
set_ui_state_notification(UI_N_SYNCHRONISATION);	
try
	{
		if ((ui_state.irp6_on_track.edp.state > 0)&&(ui_state.irp6_on_track.edp.is_synchronised == false))
		{
			ui_robot.irp6_on_track->ecp->synchronise();
			ui_state.irp6_on_track.edp.is_synchronised = ui_robot.irp6_on_track->ecp->is_synchronised();
		}
		if ((ui_state.irp6_on_track.edp.state > 0)&&(ui_state.irp6_on_track.edp.is_synchronised == true)) replySend(new Message('B','L','A',0,NULL,NULL));
	}
	CATCH_SECTION_UI
	manage_interface();	
	return 0;
}

int irp6ot_int_motion(double* v)
{
	try
	{
		if (ui_state.irp6_on_track.edp.pid!=-1)
		{
			if (ui_state.irp6_on_track.edp.is_synchronised)
			{
				for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
				{
			          irp6ot_desired_pos[i] = v[i];
		        }
			}
		}
		ui_robot.irp6_on_track->move_joints(irp6ot_desired_pos);
		irp6ot_read_joints();
		irp6ot_read_post_angle_axis();
		irp6ot_read_post_euler();
		irp6ot_read_motors();
	}
	CATCH_SECTION_UI

	return 0;
}

int irp6ot_xyz_euler_zyz_motion(double* v)
{
	try
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )
		{
			for (int i = 0; i < 7; i++) irp6ot_desired_pos[i] = v[i];
			ui_robot.irp6_on_track->move_xyz_euler_zyz(irp6ot_desired_pos);
			irp6ot_read_joints();
			irp6ot_read_post_angle_axis();
			irp6ot_read_post_euler();
			irp6ot_read_motors();
		}
    }
	CATCH_SECTION_UI
 	
	return 0;
}

int irp6ot_xyz_angle_axis_motion(double* wektor_ptgr)
{
	double wektor[8];
	double *krok;
	double wl; double l_eps = 0;
	double kx, ky, kz;
	
	try
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )
		{
			for (int i=0; i< 8; i++)
			{
				wektor[i] = wektor_ptgr[i];
			}
			
			kx = wektor[3];
			ky = wektor[4];
			kz = wektor[5];
			
			wl = sqrt(kx*kx + ky*ky + kz*kz);
			
			if((wl > 1 + l_eps) || (wl < 1 - l_eps))
			{
				wektor[3] = kx/wl;
				wektor[4] = ky/wl;
				wektor[5] = kz/wl;
			
			}
			
			for(int i=0; i<6; i++)
			{
				irp6ot_desired_pos[i] = wektor[i];
				if( i >2) irp6ot_desired_pos[i] *= wektor[6];
			}	
			
			irp6ot_desired_pos[6] = wektor[7];
			ui_robot.irp6_on_track->move_xyz_angle_axis(irp6ot_desired_pos);
			irp6ot_read_tool_angle();
			irp6ot_read_tool_euler();

		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_xyz_angle_axis_set_tool(double* wektor_ptgr)
{
	double wektor[7];
	double tool_vector[6];
	double wl; double l_eps = 0;
	double kx, ky, kz;
	
	try
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )
		{
			for (int i=0; i< 7; i++)
			{
				wektor[i] = wektor_ptgr[i];
			}	
			
			kx = wektor[3];
			ky = wektor[4];
			kz = wektor[5];
			
			wl = sqrt(kx*kx + ky*ky + kz*kz);
			
			if((wl > 1 + l_eps) || (wl < 1 - l_eps))
			{
				wektor[3] = kx/wl;
				wektor[4] = ky/wl;
				wektor[5] = kz/wl;
			}
			
			for(int i=0; i<6; i++)
			{
				tool_vector[i] = wektor[i];
				if( i >2) tool_vector[i] *= wektor[6];
			}	
		
			ui_robot.irp6_on_track->set_tool_xyz_angle_axis(tool_vector);
			irp6ot_read_tool_angle();
			irp6ot_read_tool_euler();
		}
	}
	CATCH_SECTION_UI
	return 0;
}

int irp6ot_xyz_euler_zyz_set_tool(double* v)
{
	double tool_vector[6];

	try
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )
		{
			for(int i=0; i<6; i++)
			{
				tool_vector[i] = v[i];			
			}	
			ui_robot.irp6_on_track->set_tool_xyz_euler_zyz(tool_vector);
			irp6ot_read_tool_angle();
			irp6ot_read_tool_euler();
		}
	}
	CATCH_SECTION_UI
	
	return 0;
}

int EDP_irp6_on_track_slay()
{
	if (ui_state.irp6_on_track.edp.state>0)
	{ // jesli istnieje EDP
		if (name_close(ui_state.irp6_on_track.edp.reader_fd) == -1) {
			fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n",
					__FILE__, __LINE__, strerror(errno));
		}
		delete ui_robot.irp6_on_track;

		if (SignalKill(ui_state.irp6_on_track.edp.node_nr, ui_state.irp6_on_track.edp.pid, 0, SIGTERM, 0, 0) == -1) {
			perror("UI(EDP_on_track) SignalKill()");
		};
		ui_state.irp6_on_track.edp.state = 0; // edp wylaczone		
		ui_state.irp6_on_track.edp.is_synchronised = false;
		replySend(new Message('B','K','A',0,NULL,NULL));

		ui_state.irp6_on_track.edp.pid = -1;
		ui_state.irp6_on_track.edp.reader_fd = -1;

	}
	manage_interface();
	return 0;
}

int pulse_reader_irp6ot_start()
{
	pulse_reader_irp6ot_start_exec_pulse();
		manage_interface();
	return 0;
}
int pulse_reader_irp6ot_stop()
{
	pulse_reader_irp6ot_stop_exec_pulse();
		manage_interface();
	return 0;
}
int pulse_reader_irp6ot_trigger()
{
	pulse_reader_irp6ot_trigger_exec_pulse();
		manage_interface();
	return 0;
}

int irp6ot_servo_algorithm_set(double* v)
{
	BYTE servo_alg_no_output[IRP6_ON_TRACK_NUM_OF_SERVOS];
	BYTE servo_par_no_output[IRP6_ON_TRACK_NUM_OF_SERVOS];
	
	try
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )
		{
			for(int i=0; i<IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
			{
				servo_alg_no_output[i] = (unsigned char)(v[2*i]);
				servo_par_no_output[i] = (unsigned char)(v[2*i+1]);
			}
			ui_robot.irp6_on_track->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);
			irp6ot_read_servo_algorithm();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int EDP_irp6_on_track_create()
{
	set_ui_state_notification(UI_N_PROCESS_CREATION);	
short tmp;
	char tmp_string[100];
	char tmp2_string[100];
	
	FILE* file;					// do sprawdzenia czy istnieje /net/node_name/dev/TWOJ_ROBOT
	controller_state_t robot_controller_initial_state_tmp;

	try
	{
		if (ui_state.irp6_on_track.edp.state == 0)
		{
			sprintf(tmp_string,  "/dev/name/global/%s", ui_state.irp6_on_track.edp.hardware_busy_attach_point);
			
			sprintf(tmp2_string, "/dev/name/global/%s", ui_state.irp6_on_track.edp.network_resourceman_attach_point);
			
			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if((!(ui_state.irp6_on_track.edp.test_mode)) && ( access(tmp_string, R_OK)== 0  )
				|| (access(tmp2_string, R_OK)== 0 )
			)
			{
				ui_msg.ui->message("edp_irp6_on_track already exists");
			} else {
				ui_state.irp6_on_track.edp.node_nr = config->return_node_number(ui_state.irp6_on_track.edp.node_name);
		
					ui_robot.irp6_on_track = new ui_common_robot(
					&ui_state.irp6_on_track.edp,
					*config, ui_msg.all_ecp,
					ROBOT_IRP6_ON_TRACK);

			ui_state.irp6_on_track.edp.pid = ui_robot.irp6_on_track->ecp->get_EDP_pid();
									
				if (ui_state.irp6_on_track.edp.pid<0)
				{
					fprintf( stderr, "EDP spawn failed: %s\n", strerror( errno ));
					delete ui_robot.irp6_on_track;
				} else {  // jesli spawn sie powiodl
					 tmp = 0;
				 	// kilka sekund  (~1) na otworzenie urzadzenia				 	
					while((ui_state.irp6_on_track.edp.reader_fd = name_open(ui_state.irp6_on_track.edp.network_reader_attach_point, 
						NAME_FLAG_ATTACH_GLOBAL))  < 0)						
						if((tmp++)<40) {						
							delay(50);
						} else {						
						   perror("blad odwolania do READER_OT");
						   break;
						}
						
					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)	
					ui_robot.irp6_on_track->get_controller_state(&robot_controller_initial_state_tmp);
					ui_state.irp6_on_track.edp.state = 1; // edp wlaczone reader czeka na start
					replySend(new Message('B','J','A',0,NULL,NULL));					
					ui_state.irp6_on_track.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}
	}
	CATCH_SECTION_UI

	manage_interface();
	return 0;
}

int irp6ot_kinematic_set(double* v)
{
	BYTE model_no_output;
	try
	{
		if ( ui_state.irp6_on_track.edp.is_synchronised )
		{
			model_no_output = (BYTE)(v[0]);
			ui_robot.irp6_on_track->set_kinematic(model_no_output);
			irp6ot_read_kinematic();
		}
	}
	CATCH_SECTION_UI

	return 0;
}

int pulse_ecp_irp6_on_track()
{
	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	if (ui_state.irp6_on_track.edp.is_synchronised)
	{ // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui_state.irp6_on_track.ecp.trigger_fd<0)
		{
		
			 short tmp = 0;
		 	// kilka sekund  (~1) na otworzenie urzadzenia
			// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem
	
		 	ualarm( (useconds_t)( SIGALRM_TIMEOUT), 0);
			while( (ui_state.irp6_on_track.ecp.trigger_fd = name_open(ui_state.irp6_on_track.ecp.network_trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
			{
				if (errno == EINTR) break;
				if ((tmp++)<20)
					delay(50);
				else{
				   perror("blad odwolania do ECP_TRIGGER\n");
				};
			}
			// odwolanie alarmu
			ualarm( (useconds_t)( 0), 0);
		}

		if (ui_state.irp6_on_track.ecp.trigger_fd>=0) {
			if (MsgSendPulse (ui_state.irp6_on_track.ecp.trigger_fd , sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1) {
				
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


bool pulse_reader_irp6ot_start_exec_pulse ()
{

	if (ui_state.irp6_on_track.edp.state == 1)
	{
		pulse_reader_execute( ui_state.irp6_on_track.edp.reader_fd, READER_START, 0);
		ui_state.irp6_on_track.edp.state = 2;
		return true;
	}
	
	return false;	
}



bool pulse_reader_irp6ot_stop_exec_pulse ()
{

	if (ui_state.irp6_on_track.edp.state == 2)
	{
		pulse_reader_execute( ui_state.irp6_on_track.edp.reader_fd, READER_STOP, 0);
		ui_state.irp6_on_track.edp.state = 1;
		return true;
	}
	
	return false;	
}



bool pulse_reader_irp6ot_trigger_exec_pulse ()
{

	if (ui_state.irp6_on_track.edp.state == 2)
	{
		pulse_reader_execute( ui_state.irp6_on_track.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}
	
	return false;	
}


int 
reload_irp6ot_configuration()
{
	char* tmp, *tmp1;
	char tmp_string[50];	
	char tmp2_string[3];

	// jesli IRP6 on_track ma byc aktywne
	if ((ui_state.irp6_on_track.is_active = config->return_int_value("is_irp6_on_track_active")) == 1) 
	{
		// ini_con->create_ecp_irp6_on_track (ini_con->ui->ecp_irp6_on_track_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active)
		{
			delete [] ui_state.irp6_on_track.ecp.network_trigger_attach_point;
			ui_state.irp6_on_track.ecp.network_trigger_attach_point =config->return_attach_point_name 
				(configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.irp6_on_track.ecp.section_name);

			ui_state.irp6_on_track.ecp.pid = -1;
	 		ui_state.irp6_on_track.ecp.trigger_fd = -1;
	 	}

		switch (ui_state.irp6_on_track.edp.state) 
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6_on_track (ini_con->ui->edp_irp6_on_track_section);
				
				ui_state.irp6_on_track.edp.pid = -1;
				ui_state.irp6_on_track.edp.reader_fd = -1;
				ui_state.irp6_on_track.edp.state = 0;
				
				for (int i=0; i<3; i++) 
				{
					itoa( i, tmp2_string, 10 );
	
					strcpy(tmp_string,"preset_position_");
					strcat(tmp_string, tmp2_string);
					if (config->exists(tmp_string, ui_state.irp6_on_track.edp.section_name))
					{
						tmp1 = tmp = config->return_string_value(tmp_string, ui_state.irp6_on_track.edp.section_name);
						 for (int j=0; j<8; j++)
						{
							ui_state.irp6_on_track.edp.preset_position[i][j] = strtod( tmp1, &tmp1 );
						}
						delete[] tmp;
					} else {
						 for (int j=0; j<8; j++)
						{
							ui_state.irp6_on_track.edp.preset_position[i][j] = 0.0;
						}
					}
				}

				if (config->exists("test_mode", ui_state.irp6_on_track.edp.section_name))
					ui_state.irp6_on_track.edp.test_mode = config->return_int_value("test_mode", ui_state.irp6_on_track.edp.section_name);
				else 
					ui_state.irp6_on_track.edp.test_mode = 0;
				
				delete [] ui_state.irp6_on_track.edp.hardware_busy_attach_point;
				ui_state.irp6_on_track.edp.hardware_busy_attach_point = config->return_string_value 
					("hardware_busy_attach_point", ui_state.irp6_on_track.edp.section_name);
				
				delete [] ui_state.irp6_on_track.edp.network_resourceman_attach_point;
				ui_state.irp6_on_track.edp.network_resourceman_attach_point = config->return_attach_point_name 
					(configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.irp6_on_track.edp.section_name);
					
				delete [] ui_state.irp6_on_track.edp.network_reader_attach_point;
				ui_state.irp6_on_track.edp.network_reader_attach_point = config->return_attach_point_name
					(configurator::CONFIG_SERVER, "reader_attach_point", ui_state.irp6_on_track.edp.section_name);

				delete [] ui_state.irp6_on_track.edp.node_name;
				ui_state.irp6_on_track.edp.node_name = config->return_string_value ("node_name", ui_state.irp6_on_track.edp.section_name);
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	
	} else // jesli  irp6 on_track ma byc nieaktywne
	{
			switch (ui_state.irp6_on_track.edp.state) 
		{
			case -1:
			case 0:
				ui_state.irp6_on_track.edp.state = -1;
			break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
			break;
			default:
			break;
		}
	} // end irp6_on_track

	return 1;
}

