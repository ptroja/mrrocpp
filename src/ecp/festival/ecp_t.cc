
#include <stdio.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/festival/ecp_g_festival.h"
#include "ecp_mp/ecp_mp_t_festival.h"
#include "ecp/festival/ecp_t.h"



// KONSTRUKTORY
ecp_task_festival::ecp_task_festival() : ecp_task()
{
	fg = NULL;
};
ecp_task_festival::~ecp_task_festival(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_festival::task_initialization(void) 
{
		
	fg = new festival_generator (*this);
		
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_festival::main_task_algorithm(void)
{


	sr_ecp_msg->message("ECP festival  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state (); 
			
			sr_ecp_msg->message("Order received");
			
			switch ( (ECP_FESTIVAL_STATES) mp_command.mp_package.mp_2_ecp_next_state)
			{
				case ECP_GEN_FESTIVAL:
					fg->set_phrase (mp_command.mp_package.mp_2_ecp_next_state_string);
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
	return new ecp_task_festival();
};
