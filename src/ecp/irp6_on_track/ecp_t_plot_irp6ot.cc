#include <stdio.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_plot.h"
#include "ecp/irp6_on_track/ecp_t_plot_irp6ot.h"


// KONSTRUKTORY
ecp_task_plot_irp6ot::ecp_task_plot_irp6ot(configurator &_config) : ecp_task(_config)
{

};

ecp_task_plot_irp6ot::~ecp_task_plot_irp6ot(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_plot_irp6ot::task_initialization(void) 
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_plot_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP plot irp6ot  - wcisnij start");
	ecp_wait_for_start();
	y_simple_generator ysg(*this, 8);
	ysg.sensor_m = sensor_m;
		
	for(;;)			// Wewnetrzna petla nieskonczona
	{ 
		
//					for(;;) 
//					{
//						ecp_msg->message("NOWA SERIA");
//						ecp_msg->message("Ruch");
//						ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
			Move ( ysg);
//					}
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop();
		break;
	} // koniec: for(;;) wewnetrznej	
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_plot_irp6ot(_config);
};
