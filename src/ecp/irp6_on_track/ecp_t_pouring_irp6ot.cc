// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6ot.cc - zadanie przelewania, ECP dla IRP6_ON_TRACK
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_pouring.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_t_pouring.h"
#include "ecp/irp6_on_track/ecp_t_pouring_irp6ot.h"

// KONSTRUKTORY
ecp_task_pouring_irp6ot::ecp_task_pouring_irp6ot() : ecp_task()
{
	sg = NULL;
	spg = NULL;
};

ecp_task_pouring_irp6ot::~ecp_task_pouring_irp6ot(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_pouring_irp6ot::task_initialization(void) 
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	 ecp_m_robot = new ecp_irp6_on_track_robot (*this); 
	
	sg = new ecp_smooth_generator (*this, true);
	spg = new ecp_smooth_pouring_generator(*this, true);
		
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_pouring_irp6ot::main_task_algorithm(void)
{

	int size;				  	
	char * path1;

	sr_ecp_msg->message("ECP pouring irp6ot  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state (); 
			
			sr_ecp_msg->message("Order received");
			
			switch ( (POURING_ECP_STATES) mp_command.mp_package.mp_2_ecp_next_state)
			{
				case ECP_GEN_SMOOTH:
				  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.mp_2_ecp_next_state_string);				  	
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.mp_2_ecp_next_state_string);
					sg->load_file_with_path (path1);			
					//printf("\nON_TRACK ECP_GEN_SMOOTH :%s\n\n", path1);
					delete[] path1;
					//printf("OT po delete\n");
					Move (*sg);
					//printf("OT po move\n");
					break;
				case ECP_GEN_POURING:
					size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.mp_2_ecp_next_state_string);				  	
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.mp_2_ecp_next_state_string);
					spg->load_file_with_path (path1);			
					//printf("\nON_TRACK ECP_GEN_SMOOTH :%s\n\n", path1);
					delete[] path1;
					//printf("OT po delete\n");
					Move (*spg);
					//printf("OT po move\n");
					break;
				case GRIP:
					ecp_gripper_opening(*this, -0.03, 1000);
					break;
				case LET_GO:
					ecp_gripper_opening(*this, 0.03, 1000);
					break;
				default:
				break;
			}
			
			
		} //end for
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_pouring_irp6ot();
};
