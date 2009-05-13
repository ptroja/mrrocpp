#include "ecp/irp6_on_track/ecp_t_haar_irp6ot.h"

#include <iostream>
namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

#define robot1

//Konstruktory
haar::haar(lib::configurator &_config) :
	task(_config) {
}

void haar::task_initialization(void) {

	try {
		//Wczytanie parametrow konfiguracyjnych.
		rotation = config.return_int_value("rotation"); //Czy bedzie wyznaczana rotacja?
		smooth_path = config.return_string_value("smooth_path");//Sciezka z opisem punktu startowego podawanego smooth_generatorowi


		//Create cvFraDIA sensor - for testing purposes.
		sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,
				"[vsp_cvfradia]", *this,
				sizeof(lib::sensor_image_t::sensor_union_t::deviation_t));
		//Configure sensor.
		sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

		ecp_m_robot = new ecp_irp6_on_track_robot(*this);

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
		printf("EXCEPTION caught in task_initialization.\n");
	}
}

void haar::main_task_algorithm(void) {

	//Dojazd do pozycji nad stolem.
	smooth_gen->load_file_with_path(smooth_path);
	smooth_gen->Move();

	//	//czy FraDIA ma dokonac detekcji z rotacja.
		if(rotation){
			std::cout<<"Rotacja.\n";
			rot_gripper_gen = new ecp_g_rotate_gripper(*this,0.12);
			rot_gripper_gen->sensor_m = sensor_m;
			std::cout<<"Przed move\n";
			rot_gripper_gen->Move();
			std::cout<<"Po move\n";
		}else{
			std::cout<<"bez rotacji\n";
		}

	//Czekam, az czujnik bedzie skonfigurowany.
//	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
//	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
//		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
//	}



	//Generator nadjezdzajacy nad obiekt.

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



void haar::set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
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
void haar::init_td(lib::POSE_SPECIFICATION pspec, int internode_no){
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
	return new irp6ot::task::haar(_config);

}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

