#include <stdio.h>

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_pw_irp6ot.h"

//Konstruktory
ecp_task_pw_irp6ot::ecp_task_pw_irp6ot(configurator &_config) : ecp_task(_config)
{}

ecp_task_pw_irp6ot::~ecp_task_pw_irp6ot()
{
}


void ecp_task_pw_irp6ot::task_initialization(void)
{
	ecp_m_robot=new ecp_irp6_on_track_robot(*this);
	kolo_gen=new ecp_g_pw_kolo(*this);

	sr_ecp_msg->message("ECP PW loaded");
}


void ecp_task_pw_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP pw irp6ot  - wcisnij starttt");
	ecp_wait_for_start();

	sr_ecp_msg->message("Przed MOVE");
	kolo_gen->Move();
	sr_ecp_msg->message("Po MOVE");


	sr_ecp_msg->message("przed wait_for_stop\n");
	ecp_termination_notice();
	ecp_wait_for_stop();
	sr_ecp_msg->message("po wait_for_stop\n");
	sr_ecp_msg->message("po wait_for_stop\n");
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_pw_irp6ot(_config);
}

