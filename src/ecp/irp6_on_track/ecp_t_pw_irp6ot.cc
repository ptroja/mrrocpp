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
	sr_ecp_msg->message("ECP pw irp6ot  - wcisnij start");
	ecp_wait_for_start();
	
	sr_ecp_msg->message("Przed MOVE");
	kolo_gen->Move();
	sr_ecp_msg->message("Po MOVE");
	
//	double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//	double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//	double v[MAX_SERVOS_NR]={0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 5.0};
//	double a[MAX_SERVOS_NR]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.5};
//	double coordinates[MAX_SERVOS_NR];
//	
//	coordinates[0]=0;
//	coordinates[1]=-10;
//	coordinates[2]=0;
//	coordinates[3]=0;
//	coordinates[4]=0;
//	coordinates[5]=0;
//	coordinates[6]=0;
//	coordinates[7]=0;
//	
//	
//	
//	//test part
//	
//	sgen->set_absolute();
//	sgen->load_coordinates(MOTOR,vp,vk,v,a,coordinates);
//	sgen->Move();
//	sgen->reset();
	//delete sgen;
	
//	//sgen=new ecp_smooth_generator(*this, true);
//	sgen->set_relative();
//	coordinates[0]=0;
//	coordinates[1]=-20;
//	coordinates[2]=0;
//	coordinates[3]=0;
//	coordinates[4]=0;
//	coordinates[5]=0;
//	coordinates[6]=0;
//	coordinates[7]=0;
//	sgen->load_coordinates(MOTOR,vp,vk,v,a,coordinates);
//		coordinates[0]=0;
//	coordinates[1]=0;
//	coordinates[2]=0;
//	coordinates[3]=10;
//	coordinates[4]=0;
//	coordinates[5]=0;
//	coordinates[6]=0;
//	coordinates[7]=0;
//	sgen->load_coordinates(MOTOR,vp,vk,v,a,coordinates);
//		coordinates[0]=0;
//	coordinates[1]=0;
//	coordinates[2]=0;
//	coordinates[3]=10;
//	coordinates[4]=0;
//	coordinates[5]=0;
//	coordinates[6]=0;
//	coordinates[7]=0;
//	sgen->load_coordinates(MOTOR,vp,vk,v,a,coordinates);
//		coordinates[0]=0;
//	coordinates[1]=0;
//	coordinates[2]=0;
//	coordinates[3]=10;
//	coordinates[4]=0;
//	coordinates[5]=0;
//	coordinates[6]=0;
//	coordinates[7]=0;
//	sgen->load_coordinates(MOTOR,vp,vk,v,a,coordinates);
//	sgen->Move();
//	delete sgen;
//	
//	
//	//sgen=new ecp_smooth_generator(*this, true);
//	sgen->set_absolute();
//	sgen->load_coordinates(MOTOR,0,-30,0,0,0,0,0,0);
//	sgen->Move();
//	sgen->reset();
//	//delete sgen;
//	
//	//sgen=new ecp_smooth_generator(*this, true);
//	sgen->set_relative();
//	sgen->load_coordinates(MOTOR,0,-20,0,0,0,0,0,0);
//	sgen->load_coordinates(MOTOR,0,0,0,10,0,0,0,0);
//	sgen->load_coordinates(MOTOR,0,0,0,10,0,0,0,0);
//	sgen->load_coordinates(MOTOR,0,0,0,10,0,0,0,0);
//	sgen->Move();
//	delete sgen;
//	
//	
//	sgen=new ecp_smooth_generator(*this, true);
//	sgen->set_absolute();
//	sgen->load_coordinates(MOTOR,0,0,0,0,0,0,0,0);
//	sgen->Move();
//	delete sgen;


	

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

