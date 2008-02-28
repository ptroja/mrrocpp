// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow 
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <stdio.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/festival/ecp_local.h"
#include "ecp/festival/ecp_g_festival.h"
#include "ecp/festival/ecp_t.h"



// KONSTRUKTORY
ecp_task_festival::ecp_task_festival() : ecp_task()
{
	gt = NULL;
	nrg = NULL;
	rgg = NULL;
	gag = NULL;
	rfrg = NULL;
	tig = NULL;
	sg = NULL;
};
ecp_task_festival::~ecp_task_festival(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_festival::task_initialization(void) 
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	 ecp_m_robot = new ecp_festival_robot (*this); 
		
	fg = new ecp_festival_generator (*this);
		
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_festival::main_task_algorithm(void)
{

	int size;				  	
	char * path1;

	sr_ecp_msg->message("ECP rcsc festival  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state (); 
			
			sr_ecp_msg->message("Order received");
			
			switch ( (ECP_FESTIVAL_STATES) mp_command.mp_package.mp_2_ecp_next_state)
			{
				case ECP_GEN_FESTIVAL:
				  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.mp_2_ecp_next_state_string);				  	
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.mp_2_ecp_next_state_string);
					sg->load_file_with_path (path1);
	//				printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
					delete[] path1;
					Move (*fg);
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
	return new ecp_task_rcsc_irp6ot();
};
