// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6p.cc - zadanie przelewania, ECP dla IRP6_POSTUMENT
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

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_pouring.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/irp6_postument/ecp_t_pouring_irp6p.h"



// KONSTRUKTORY
ecp_task_pouring_irp6p::ecp_task_pouring_irp6p() : ecp_task()
{
	sg = NULL;
};

ecp_task_pouring_irp6p::~ecp_task_pouring_irp6p(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_pouring_irp6p::task_initialization(void) 
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	 ecp_m_robot = new ecp_irp6_postument_robot (*this); 
	
	// powolanie czujnikow
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
			
			switch ( (POURING_ECP_STATES) mp_command.mp_package.mp_2_ecp_next_state)
			{
				case ECP_GEN_SMOOTH:
					//printf("P w ECP\n");
				  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.mp_2_ecp_next_state_string);				  	
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.mp_2_ecp_next_state_string);
					//printf("\n1 POSTUMENT ECP_GEN_SMOOTH :%s\n\n", path1);
					sg->load_file_with_path (path1);
					//printf("\nPOSTUMENT ECP_GEN_SMOOTH :%s\n\n", path1);
					delete[] path1;
					//printf("P po delete\n");
					Move (*sg);
					//printf("P po move\n");
					break;
				case GRIP:
					ecp_gripper_opening(*this, -0.018, 1000);
					break;
				case LET_GO:
					ecp_gripper_opening(*this, 0.014, 1000);
					break;
				default:
				break;
			} // end switch
			
			
		} //end for
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_pouring_irp6p();
};
