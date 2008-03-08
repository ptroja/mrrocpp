// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6p.cc - zadanie przelewania, ECP dla IRP6_POSTUMENT
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_pouring.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_pouring.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/irp6_postument/ecp_t_pouring_irp6p.h"



// KONSTRUKTORY
ecp_task_pouring_irp6p::ecp_task_pouring_irp6p(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
};

ecp_task_pouring_irp6p::~ecp_task_pouring_irp6p(){};

void ecp_task_pouring_irp6p::task_initialization(void) 
{
	 ecp_m_robot = new ecp_irp6_postument_robot (*this); 

	// Powolanie czujnikow
//	sensor_m[SENSOR_FORCE_POSTUMENT] = 
//		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);
				
/*	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
*/
	sg = new ecp_smooth_generator (*this, true);
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_pouring_irp6p::main_task_algorithm(void)
{

	int size;				  	
	char * path1;
	
	sr_ecp_msg->message("ECP pouring irp6p  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state (); 
			
			sr_ecp_msg->message("Order received");
			
			switch ( (POURING_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state)
			{
				case ECP_GEN_SMOOTH:
				  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);				  	
					path1 = new char[size];
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					sg->load_file_with_path (path1);
					delete[] path1;
					Move (*sg);
					break;
				case GRIP:
					ecp_gripper_opening(*this, -0.018, 1000);
					break;
				case LET_GO:
					ecp_gripper_opening(*this, 0.015, 2000);
					break;
				case WEIGHT:
					printf("force0: %d\n", sensor_m.begin()->second->image.force.rez[0]);
					printf("force1: %d\n", sensor_m.begin()->second->image.force.rez[0]);
					printf("force2: %d\n", sensor_m.begin()->second->image.force.rez[0]);
					break;
				default:
				break;
			} // end switch
			ecp_termination_notice();
			
		} //end for
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_pouring_irp6p(_config);
};
