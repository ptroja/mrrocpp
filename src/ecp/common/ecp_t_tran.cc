// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow 
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_mechatronika/ecp_local.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/speaker/ecp_local.h"

#include "ecp/common/ecp_t_tran.h"
#include "ecp/common/ecp_generator_t.h"


// KONSTRUKTORY
ecp_task_tran::ecp_task_tran() : ecp_task(){};
ecp_task_tran::~ecp_task_tran(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_tran::task_initialization(void) 
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (strcmp(config->section_name, "[ecp_irp6_on_track]") == 0)
		{ ecp_m_robot = new ecp_irp6_on_track_robot (*this); }
	else if (strcmp(config->section_name, "[ecp_irp6_postument]") == 0)
		{ ecp_m_robot = new ecp_irp6_postument_robot (*this); }
	else if (strcmp(config->section_name, "[ecp_conveyor]") == 0)
		{ ecp_m_robot = new ecp_conveyor_robot (*this); }
	else if (strcmp(config->section_name, "[ecp_speaker]") == 0)
		{ ecp_m_robot = new ecp_speaker_robot (*this); }
	else if (strcmp(config->section_name, "[ecp_irp6_mechatronika]") == 0)
		{ ecp_m_robot = new ecp_irp6_mechatronika_robot (*this); }
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_tran::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP transparentne - wcisnij start");
	
	ecp_wait_for_start();

	ecp_generator_t gt (*this, true);
	
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("Ruch");

			Move (gt);
		}
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (void)
{
	return new ecp_task_tran();
};
