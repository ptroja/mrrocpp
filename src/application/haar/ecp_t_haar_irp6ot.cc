#include "ecp_t_haar_irp6ot.h"

#include <iostream>
#include <cstdio>
#include <unistd.h>
namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

#define robot1

#define JAW_PINCHING_0 -0.01//zacisk szczeki dla puszki
#define JAW_PINCHING_1 -0.02//zacisk szczeki dla pudelka
#define LOWERNIG_INTERVAL_0 -0.073 //interwal co ktory wlaczany jest serwomechanizm w plaszczyznie							//dla puszki
#define LOWERNIG_INTERVAL_1 -0.05
#define GAGEN_INTERVAL_0 500 // ustawienie generatora gripper approach
#define GAGEN_INTERVAL_1 401

//Konstruktory
haar::haar(lib::configurator &_config) :
	task(_config) {
	//Wczytanie parametrow konfiguracyjnych.
	rotation = config.value<int> ("rotation"); //Czy bedzie wyznaczana rotacja?
	smooth_path = config.value<std::string> ("smooth_path");//Sciezka z opisem punktu startowego podawanego smooth_generatorowi
	object_type = config.value<int> ("object_type");

	if (object_type == 0)//Ustawiamy zadanie dla puszki.
	{
		jaw_pinching = JAW_PINCHING_0;
		lowering_interval = LOWERNIG_INTERVAL_0;
		ga_gen_interval = GAGEN_INTERVAL_0;
	} else if (object_type == 1) {
		jaw_pinching = JAW_PINCHING_1;
		lowering_interval = LOWERNIG_INTERVAL_1;
		ga_gen_interval = GAGEN_INTERVAL_1;
	} else {
		sr_ecp_msg->message(
				"Zle zdefiniowany obiekt: USTAW PLIK KONFIGURACYJNY");
		ecp_termination_notice();
		ecp_wait_for_stop();
	}

	//Create cvFraDIA sensor - for testing purposes.
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = new fradia_sensor_haar_detect(this->config, "[vsp_cvfradia]");

	//Configure sensor.
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA]->configure_sensor();

	ecp_m_robot = new robot(*this);

	planar_vis = new generator::ecp_vis_ib_eih_planar_irp6ot(*this);
	planar_vis->sensor_m = sensor_m;

	//Smooth generator
	//smooth_gen = new common::generator::newsmooth(*this, true);
	bef_gen = new common::generator::bias_edp_force(*this);
	//gripper approach constructor (task&, no_of_steps)
	ga_gen = new common::generator::tff_gripper_approach(*this, 8);
	//Linear generator.
	linear_gen = NULL;

	sr_ecp_msg->message("ecp PW loaded");
}

void haar::main_task_algorithm(void) {
	/*
	 //Dojazd do pozycji nad stolem.
	 smooth_gen->load_file_with_path(smooth_path.c_str());
	 smooth_gen->Move();

	 //Czy FraDIA ma dokonac detekcji z rotacja.
	 if (rotation) {
	 sr_ecp_msg->message("Rotacja");

	 rot_gripper_gen = new ecp_g_rotate_gripper(*this, 0.36);
	 rot_gripper_gen->sensor_m = sensor_m;
	 rot_gripper_gen->Move();
	 } else {
	 sr_ecp_msg->message("BEZ Rotacja");
	 }

	 sr_ecp_msg->message("Centrowanie");
	 //Centrowanie wykonuje w kilku krokach.
	 init_td(lib::XYZ_ANGLE_AXIS, 1000+9*object_type);
	 for (int i = 0; i< 3; i++){
	 //Generator nadjezdzajacy nad obiekt.
	 planar_vis->Move();
	 if (i == 0 && !planar_vis->above_object)
	 i--; //W pierwszym etapie nadjezdzania wywoujemy generator do skutku;
	 else{
	 set_td_coordinates(0.0, 0.0, lowering_interval, 0.0, 0.0, 0.0, 0.0);
	 //Opuszczamy manilpulator.
	 linear_gen=new common::generator::linear(*this,td,1);
	 linear_gen->Move();
	 delete linear_gen;
	 }
	 }

	 bef_gen->Move();
	 //Configuration of gripper approach configure(speed, time_period).
	 ga_gen->configure(0.02, ga_gen_interval);
	 ga_gen->Move();

	 //Inicjalizacja td.
	 init_td(lib::XYZ_ANGLE_AXIS, 400);

	 sr_ecp_msg->message("linear_gen\n");
	 //Podniesienie chwytaka o 0.4 cm.
	 set_td_coordinates(0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0);
	 linear_gen=new common::generator::linear(*this,td,1);
	 linear_gen->Move();
	 delete linear_gen;

	 //Zacisniecie szczek.
	 set_td_coordinates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, jaw_pinching);
	 linear_gen=new common::generator::linear(*this,td,1);
	 linear_gen->Move();
	 delete linear_gen;

	 //Podniesienie o 2cm.
	 set_td_coordinates(0,0,0.04,0,0,0,0);
	 linear_gen=new common::generator::linear(*this,td,1);
	 linear_gen->Move();
	 delete linear_gen;

	 sr_ecp_msg->message("Wait_for_stop\n");
	 ecp_termination_notice();

	 ecp_wait_for_stop();
	 */
}

void haar::set_td_coordinates(double cor0, double cor1, double cor2,
		double cor3, double cor4, double cor5, double cor6) {
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
void haar::init_td(lib::ECP_POSE_SPECIFICATION pspec, int internode_no) {
	td.arm_type = pspec;
	td.interpolation_node_no = 1;
	td.internode_step_no = internode_no; //motion time
	td.value_in_step_no = internode_no - 2; //motion time-2 ??
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::haar(_config);

}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

