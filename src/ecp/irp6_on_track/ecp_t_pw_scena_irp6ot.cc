#include "ecp/irp6_on_track/ecp_t_pw_scena_irp6ot.h"

#include <iostream>
#define robot1

//Konstruktory
ecp_task_pw_scena_irp6ot::ecp_task_pw_scena_irp6ot(configurator &_config) :
	ecp_task(_config) {
}

ecp_task_pw_scena_irp6ot::~ecp_task_pw_scena_irp6ot() {
}

void ecp_task_pw_scena_irp6ot::task_initialization(void) {

	try {
		// Create cvFraDIA sensor - for testing purposes.
		sensor_m[SENSOR_CVFRADIA] = new ecp_mp_cvfradia_sensor(SENSOR_CVFRADIA,
				"[vsp_cvfradia]", *this,
				sizeof(sensor_image_t::sensor_union_t::fradia_t));
		//Configure sensor.
		sensor_m[SENSOR_CVFRADIA]->configure_sensor();
		ecp_m_robot = new ecp_irp6_on_track_robot(*this);
		//Generator ruchu dla rozpoznawania sceny.
		scena_gen = new ecp_g_pw_scena(*this);
		scena_gen->sensor_m = sensor_m;
		//Smooth generator
		smooth_gen = new ecp_smooth_generator(*this, true);

		sr_ecp_msg->message("ECP PW loaded");
	} catch (...) {
		printf("EXCEPTION\n");
	}
}

void ecp_task_pw_scena_irp6ot::main_task_algorithm(void) {
	sr_ecp_msg->message("ECP pw irp6ot  - wcisnij start");
	ecp_wait_for_start();

	//#ifdef robot1
	//	smooth_gen->load_file_with_path("/net/robot1/home/pwilkows/workspace/mrrocpp/trj/nad_stolem_joint.trj");
	//#else
	//	smooth_gen->load_file_with_path("/net/qnx_pw/home/pwilkows/workspace/mrrocpp/trj/nad_stolem_joint.trj");
	//#endif

	//smooth_gen->Move();
	//sr_ecp_msg->message("Wczytano trajektorie.\n");


	//	std::cout<< "Odp z fradii??: " << sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report << std::endl;


	//	if( sensor_m[SENSOR_CVFRADIA]->from_vsp.vsp_report == VSP_REPLY_OK )
	//		sr_ecp_msg->message("otrzymano VSP_REPLY_OK");

	scena_gen->Move();

	sr_ecp_msg->message("przed wait_for_stop\n");
	ecp_termination_notice();
	ecp_wait_for_stop();
	sr_ecp_msg->message("po wait_for_stop\n");
	sr_ecp_msg->message("po wait_for_stop\n");
}

ecp_task* return_created_ecp_task(configurator &_config) {
	return new ecp_task_pw_scena_irp6ot(_config);
}

