// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow 
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/speaker/ecp_local.h"
#include "ecp/speaker/ecp_t_rcsc_speaker.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

// KONSTRUKTORY
ecp_task_rcsc_speaker::ecp_task_rcsc_speaker(configurator &_config) : ecp_task(_config)
{
	gt = NULL;
	speak = NULL;
};
ecp_task_rcsc_speaker::~ecp_task_rcsc_speaker(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_rcsc_speaker::task_initialization(void) 
{
	ecp_m_robot = new ecp_speaker_robot (*this);

	gt = new ecp_generator_t(*this, true);
	speak = new speaking_generator (*this, 8);
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_rcsc_speaker::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP rcsc speaker  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {

		sr_ecp_msg->message("Waiting for MP order");
		
		get_next_state (); 
		
		sr_ecp_msg->message("Order received");

			switch ( (RCSC_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state)
			{
				case ECP_GEN_TRANSPARENT:
					Move (*gt);
				break;
				case ECP_GEN_SPEAK:
					speak->configure(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					Move ( *speak );
				break;
				default:
				break;
			}
		
		}
		
		// Oczekiwanie na STOP
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_rcsc_speaker(_config);
};
