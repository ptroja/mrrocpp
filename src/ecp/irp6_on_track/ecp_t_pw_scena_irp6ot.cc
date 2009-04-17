#include "ecp/irp6_on_track/ecp_t_pw_scena_irp6ot.h"

#include <iostream>
namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

#define robot1

//Konstruktory
pw_scena::pw_scena(lib::configurator &_config) :
	task(_config) {
}

void pw_scena::task_initialization(void) {

	try {
		//Create cvFraDIA sensor - for testing purposes.
		sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,
				"[vsp_cvfradia]", *this,
				sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
		//Configure sensor.
		sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();


		ecp_m_robot = new ecp_irp6_on_track_robot(*this);
//

		//Generator ruchu dla rozpoznawania sceny.
		scena_gen = new generator::pw_scena(*this);
		scena_gen->sensor_m = sensor_m;

		planar_vis = new ecp_vis_ib_eih_planar_irp6ot(*this);
		planar_vis->sensor_m = sensor_m;

		//Smooth generator
		smooth_gen = new common::generator::smooth(*this, true);

		bef_gen=new common::generator::bias_edp_force(*this);
		//gripper approach constructor (task&, no_of_steps)
		ga_gen=new common::generator::tff_gripper_approach (*this, 8);
		//Linear generator.
		linear_gen=NULL;

		sr_ecp_msg->message("ECP PW loaded");
	} catch (...) {
		printf("EXCEPTION\n");
	}
}

void pw_scena::main_task_algorithm(void) {

	//Dojazd do pozycji nad stolem.
	#ifdef robot1
		smooth_gen->load_file_with_path("/net/robot1/home/pwilkows/workspace/mrrocpp/trj/nad_stolem_joint.trj");
	#else
		smooth_gen->load_file_with_path("/net/qnx_pw/home/pwilkows/workspace/mrrocpp/trj/nad_stolem_joint.trj");
	#endif

	smooth_gen->Move();

	//Czekam, az czujnik bedzie skonfigurowany.
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	}

	//Generator nadjezdzajacy nad obiekt.
	//scena_gen->Move();
	sr_ecp_msg->message("Przed planar_vis");
	planar_vis->Move();
	sr_ecp_msg->message("Po planar_vis");

	sr_ecp_msg->message("przed befgen\n");
	//Kalibracja sily..
	bef_gen->Move();

	sr_ecp_msg->message("przed gagen\n");
	//Configuration of gripper approach configure(speed, time_period).
	ga_gen->configure(0.02, 900);
	ga_gen->Move();

	//Inicjalizacja td.
	init_td(lib::XYZ_ANGLE_AXIS, 500);

	sr_ecp_msg->message("linear_gen\n");
	//Podniesienie chwytaka o 0.4 cm.
	set_td_coordinates(0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this,td,1);
	linear_gen->Move();
	delete linear_gen;

	//Zacisniecie szczek.
	set_td_coordinates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.024);
	linear_gen=new common::generator::linear(*this,td,1);
	linear_gen->Move();
	delete linear_gen;

	//Podniesienie o 2cm.
	set_td_coordinates(0,0,0.02,0,0,0,0);
	linear_gen=new common::generator::linear(*this,td,1);
	linear_gen->Move();
	delete linear_gen;

	sr_ecp_msg->message("Wait_for_stop\n");
	ecp_termination_notice();

	ecp_wait_for_stop();

}



void pw_scena::set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	td.coordinate_delta[0] = cor0; // przyrost wspolrzednej X
	td.coordinate_delta[1] = cor1;// przyrost wspolrzednej Y
	td.coordinate_delta[2] = cor2; // przyrost wspolrzednej Z
	td.coordinate_delta[3] = cor3; // przyrost wspolrzednej FI
	td.coordinate_delta[4] = cor4; // przyrost wspolrzednej TETA
	td.coordinate_delta[5] = cor5; // przyrost wspolrzednej PSI
	td.coordinate_delta[6] = cor6; // przyrost wspolrzednej PSI
}

//inicjacja struktury td - trajectory description
void pw_scena::init_td(lib::POSE_SPECIFICATION pspec, int internode_no){
	td.arm_type=pspec;
	td.interpolation_node_no=1;
	td.internode_step_no=internode_no;	//motion time
	td.value_in_step_no=internode_no-2;			//motion time-2 ??
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot::task::pw_scena(_config);
	
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

