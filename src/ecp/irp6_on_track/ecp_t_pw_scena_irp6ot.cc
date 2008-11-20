#include "ecp/irp6_on_track/ecp_t_pw_scena_irp6ot.h"

#include <iostream>

//Konstruktory
ecp_task_pw_scena_irp6ot::ecp_task_pw_scena_irp6ot(configurator &_config) : ecp_task(_config)
{}

ecp_task_pw_scena_irp6ot::~ecp_task_pw_scena_irp6ot()
{
}


void ecp_task_pw_scena_irp6ot::task_initialization(void)
{

	try {
		// Create cvFraDIA sensor - for testing purposes.
		sensor_m[SENSOR_CVFRADIA] = new ecp_mp_cvfradia_sensor(SENSOR_CVFRADIA,
				"[vsp_cvfradia]", *this, sizeof(sensor_image_t::sensor_union_t::fradia_t));
		//Configure sensor.
		sensor_m[SENSOR_CVFRADIA]->configure_sensor();
		ecp_m_robot=new ecp_irp6_on_track_robot(*this);
		//Generator ruchu dla rozpoznawania sceny.
		scena_gen=new ecp_g_pw_scena(*this);
		scena_gen->sensor_m = sensor_m;
		//Smooth generator
		smooth_gen = new ecp_smooth_generator(*this, true);

		sr_ecp_msg->message("ECP PW loaded");
	} catch (...) {
		printf("EXCEPTION\n");
	}
}


void ecp_task_pw_scena_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP pw irp6ot  - wcisnij start");
	ecp_wait_for_start();

	smooth_gen->load_file_with_path("/net/lenin/home/pwilkows/workspace/mrrocpp/trj/nad_stolem_joint.trj");
	smooth_gen->Move();
	sr_ecp_msg->message("Wczytano trajektorie.\n");

	sr_ecp_msg->message("Czekam na odpowidz z fradii.");

	sensor_m[SENSOR_CVFRADIA]->get_reading();
	while( sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report == VSP_SENSOR_NOT_CONFIGURED )
	{
		sensor_m[SENSOR_CVFRADIA]->get_reading();
		sr_ecp_msg->message("Czekam na fradie");
	}

	std::cout<< "Odp z fradii??: " << sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report << std::endl;

	sr_ecp_msg->message("Fradia skonfigurowana.");

	if( sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report == VSP_REPLY_OK )
		sr_ecp_msg->message("otrzymano VSP_REPLY_OK");

	//scena_gen->Move();

	sr_ecp_msg->message("przed wait_for_stop\n");
	ecp_termination_notice();
	ecp_wait_for_stop();
	sr_ecp_msg->message("po wait_for_stop\n");
	sr_ecp_msg->message("po wait_for_stop\n");
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_pw_scena_irp6ot(_config);
}

