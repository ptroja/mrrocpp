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
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_ppg.h"
#include "ecp_mp/ecp_mp_tr_player.h"
#include "ecp/common/ecp_t_playerpos.h"

// KONSTRUKTORY
ecp_task_playerpos::ecp_task_playerpos() : ecp_task()
{
	ppg = NULL;
};
ecp_task_playerpos::~ecp_task_playerpos(){};

// methods for ECP template to redefine in concrete classes
void ecp_task_playerpos::task_initialization(void) 
{
	//ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	
	sr_ecp_msg->message("ECP loaded");

	transmitter_m[TRANSMITTER_PLAYER] = 
           new player_transmitter (TRANSMITTER_PLAYER, "[transmitter_player]", *this,
                   "192.168.1.64", 6665, "position", 1, 'a');
                   
   	ppg = new playerpos_generator(*this, 8);
	ppg->transmitter_m = transmitter_m;
};

void ecp_task_playerpos::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP playerpos - wcisnij start");
	ecp_wait_for_start();
	for(;;)			// Wewnetrzna petla nieskonczona
	{ 
		for(;;) 
		{
			Move ( *ppg);
		}
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop();
		break;
	} // koniec: for(;;) wewnetrznej
};

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_playerpos();
};
