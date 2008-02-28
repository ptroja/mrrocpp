#include <stdio.h>

#include "ecp/common/ecp_g_playerpos.h"
#include "ecp/common/ecp_t_playerpos.h"

ecp_task_playerpos::ecp_task_playerpos() : ecp_task()
{
	ppg = NULL;
}

ecp_task_playerpos::~ecp_task_playerpos(){}

void ecp_task_playerpos::task_initialization(void) 
{
	sr_ecp_msg->message("ECP loaded");
               
   	ppg = new playerpos_generator(*this);
}

void ecp_task_playerpos::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP playerpos - wcisnij start");
	ecp_wait_for_start();
	for(;;) {
		for(;;) {
			Move ( *ppg);
		}
		
		// Oczekiwanie na STOP
		ecp_wait_for_stop();
		break;
	}
}

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_playerpos();
}
