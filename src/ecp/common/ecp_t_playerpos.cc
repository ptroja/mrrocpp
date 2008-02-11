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

#include "ecp/common/ecp_g_playerpos.h"
#include "ecp_mp/ecp_mp_tr_player.h"
#include "ecp/common/ecp_t_playerpos.h"

ecp_task_playerpos::ecp_task_playerpos() : ecp_task()
{
	ppg = NULL;
}

ecp_task_playerpos::~ecp_task_playerpos(){}

void ecp_task_playerpos::task_initialization(void) 
{
	sr_ecp_msg->message("ECP loaded");

	transmitter_m[TRANSMITTER_PLAYER] = 
           new player_transmitter (TRANSMITTER_PLAYER, "[transmitter_player]", *this,
                   "192.168.1.64", 6665, "position", 1, 'a');
                   
   	ppg = new playerpos_generator(*this);
	ppg->transmitter_m = transmitter_m;
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
		printf("przed wait for stop\n");
		ecp_wait_for_stop();
		break;
	}
}

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_playerpos();
}
