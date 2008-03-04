/*	! \file src/edp/irp6s/force.cc
    * \brief WATKI SILOWE
    * Ostatnia modyfikacja: kwiecieñ 2006*/

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <devctl.h>
#include <string.h>
#include <signal.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <sys/netmgr.h>
#include <semaphore.h>
#include <fstream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "edp/common/edp.h"
#include "edp/common/edp_force_sensor.h"
	
/********************************* GLOBALS **********************************/

sr_vsp *sr_msg;		//!< komunikacja z SR

static sem_t new_ms; //!< semafor dostepu do nowej wiadomosci dla vsp

			
edp_force_sensor *vs;

static bool TERMINATE=false;			//!< zakonczenie obydwu watkow

//!< watek do komunikacji miedzy edp a vsp


void * edp_irp6s_postument_track_effector::edp_vsp_thread_start(void* arg)
{
	 static_cast<edp_irp6s_postument_track_effector*> (arg)->edp_vsp_thread(arg);
}



void * edp_irp6s_postument_track_effector::edp_vsp_thread(void *arg)
 {
	name_attach_t *edp_vsp_attach;	 
	uint64_t e;     //!< kod bledu systemowego
	int vsp_caller;            //!< by Y&W
	VSP_EDP_message vsp_edp_command;
	EDP_VSP_reply edp_vsp_reply;
	
	set_thread_priority(pthread_self() , MAX_PRIORITY-4);
	
	//!< zarejestrowanie nazwy identyfikujacej serwer
	
	if ((edp_vsp_attach = name_attach(NULL, config.return_attach_point_name(configurator::CONFIG_SERVER, "edp_vsp_attach_point"),
		NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach EDP_VSP\n");
		sr_msg->message (SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
	}
	long counter = 0;
	while(1) 
	{	
		vsp_caller = MsgReceive(edp_vsp_attach->chid, &vsp_edp_command, sizeof(vsp_edp_command), NULL);
		
		if (vsp_caller == -1) /*!Error condition, exit */
		{
			e = errno;
			perror("EDP_VSP: Receive from VSP failed\n");
			sr_msg->message (SYSTEM_ERROR, e, "EDP: Receive from VSP failed");
			break;
		}
	
		if (vsp_caller == 0) /*!Pulse received */
		{		
			switch (vsp_edp_command.hdr.code)
			{
				case _PULSE_CODE_DISCONNECT:
					ConnectDetach(vsp_edp_command.hdr.scoid);
				break;
				case _PULSE_CODE_UNBLOCK:
				break;
				default:
				break;
			}
			continue;
		}
		
		/*!A QNX IO message received, reject */
		if (vsp_edp_command.hdr.type >= _IO_BASE && vsp_edp_command.hdr.type <= _IO_MAX)
		{		
			MsgReply(vsp_caller, EOK, 0, 0);			
			continue;
		}
		
		if (vsp_edp_command.konfigurowac)
			force_sensor_do_configure = true;//!< jesli otrzymano od VSP polecenie konfiguracji czujnika
		//!< oczekiwanie nowego pomiaru			
		sem_wait(&new_ms);
		//!< przygotowanie struktury do wyslania
		
		// uwaga sila nie przemnozona przez tool'a i current frame orientation						
		force_msr_download(edp_vsp_reply.force, 0);
		
		counter++;	
									
		rb_obj->lock_mutex();
		edp_vsp_reply.servo_step=rb_obj->step_data.step;
		for (int i=0;i<=5;i++)
		{
			edp_vsp_reply.current_present_XYZ_ZYZ_arm_coordinates[i]=rb_obj->step_data.current_kartez_position[i];  
		}		
		rb_obj->unlock_mutex();
		
		//!< wyslanie danych
		if ( MsgReply(vsp_caller, EOK, &edp_vsp_reply, sizeof(edp_vsp_reply)) ==-1) //!< by Y&W
		{
			e = errno;
			perror("EDP_VSP: Reply to VSP failed\n");
			sr_msg->message (SYSTEM_ERROR, e, "EDP: Reply to VSP failed");
		}
	} //!< end while	
	return 0;
}

void * edp_irp6s_postument_track_effector::force_thread_start(void* arg)
{
	 static_cast<edp_irp6s_postument_track_effector*> (arg)->force_thread(arg);
}



//!< watek do komunikacji ze sprzetem
void * edp_irp6s_postument_track_effector::force_thread(void *arg)
{

	set_thread_priority(pthread_self() , MAX_PRIORITY-1);
	sem_init( &new_ms, 0, 0);      
	/*!Lokalizacja procesu wywietlania komunikatow SR */ 
	if ((sr_msg = new sr_vsp(EDP, config.return_attach_point_name(configurator::CONFIG_SERVER, "edp_vsp_attach_point"),	
		config.return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]"))) == NULL) 
	{
		printf("communication with SR not ready\n");
	}	
	vs = return_created_edp_force_sensor();		//!< czujnik wirtualny
	
	try{
		vs->configure_sensor();
	}
		
	catch (sensor::sensor_error e){
		printf("sensor_error w force thread EDP\n");
		
		switch(e.error_no){
			case SENSOR_NOT_CONFIGURED:
				vs->from_vsp.vsp_report=VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				vs->from_vsp.vsp_report=VSP_READING_NOT_READY;
				break;
		}; //!< end switch
		sr_msg->message (FATAL_ERROR, e.error_no);
		
	} //!< end CATCH
	
	catch(...) {
		printf("unidentified error force thread w EDP\n");
	}
	
	while(!TERMINATE) //!< for (;;)
	{
		try{
		
		if (force_sensor_do_configure){ //!< jesli otrzymano polecenie konfiguracji czujnika		
			vs->configure_sensor();			
			force_sensor_do_configure = false;	//!< ustawienie flagi ze czujnik jest ponownie skonfigurowany
			sem_trywait(&new_ms);
			sem_post(&new_ms);	 //!< jest gotowy nowy pomiar	
		} 
		else {	
				//!< cout << "przed Wait for event" << endl;		
				vs->wait_for_event();
				//!< cout << "po Wait for event" << endl;
		
				vs->initiate_reading();
		//!< 		cout << "Initiate reading" << endl;
				if (force_sensor_do_configure == false) {
					sem_trywait(&new_ms);
					sem_post(&new_ms);	 //!< jest gotowy nowy pomiar				
				}
		}
		
		} //!< koniec TRY

	catch (sensor::sensor_error e){
			printf("sensor_error w force thread  EDP\n");
			
			switch(e.error_no){
			case SENSOR_NOT_CONFIGURED:
				vs->from_vsp.vsp_report=VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				vs->from_vsp.vsp_report=VSP_READING_NOT_READY;
				break;
			}; //!< end switch
		sr_msg->message (FATAL_ERROR, e.error_no);
			
		} //!< end CATCH
		
		catch(...) {
			printf("unidentified error force thread w EDP\n");
		};
		
	}		//!< //!< end while(;;)

	return NULL;
} //!< end MAIN
