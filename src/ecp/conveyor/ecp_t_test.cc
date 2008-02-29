#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_t_test.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/conveyor/ecp_g_test.h"

// KONSTRUKTORY
ecp_task_conveyor_test::ecp_task_conveyor_test() : ecp_task()
{}

ecp_task_conveyor_test::~ecp_task_conveyor_test()
{}

// methods for ECP template to redefine in concrete classes
void ecp_task_conveyor_test::task_initialization(void)
{
	ecp_m_robot = new ecp_conveyor_robot (*this);

	sr_ecp_msg->message("ECP loaded");
}


void ecp_task_conveyor_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP rcsc speaker  - wcisnij start");
	ecp_wait_for_start();
	y_simple_generator ysg(*this, 8);
	ysg.sensor_m = sensor_m;

	for(;;) { // Wewnetrzna petla nieskonczona

		for(;;) {
			sr_ecp_msg->message("NOWA SERIA");
			sr_ecp_msg->message("Ruch");
			sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
			Move (ysg);
		}

		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
}

ecp_task* return_created_ecp_task (void)
                {
	                return new ecp_task_conveyor_test();
                }
