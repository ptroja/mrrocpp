#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_t_legobrick_conv.h"
#include "ecp/conveyor/ecp_g_legobrick.h"

// KONSTRUKTORY
ecp_task_conveyor_lego_brick::ecp_task_conveyor_lego_brick(configurator &_config) : ecp_task(_config)
{}

ecp_task_conveyor_lego_brick::~ecp_task_conveyor_lego_brick()
{}

// methods for ECP template to redefine in concrete classes
void ecp_task_conveyor_lego_brick::task_initialization(void)
{
	ecp_m_robot = new ecp_conveyor_robot (*this);

	sr_ecp_msg->message("ECP loaded");
}


void ecp_task_conveyor_lego_brick::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP lego brick speaker  - wcisnij start");
	ecp_wait_for_start();
	conveyor_incremental_move ysg(*this, 100);
	ysg.sensor_m = sensor_m;

	for(;;) { // Wewnetrzna petla nieskonczona

		for(;;) {
			sr_ecp_msg->message("NOWA SERIA");
			sr_ecp_msg->message("Ruch");
			sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");

			ysg.Move();
		}

		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_conveyor_lego_brick(_config);
}
