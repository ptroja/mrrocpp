#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_postument/ecp_t_eihcalibration_irp6p.h"
#include <unistd.h>

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

//Constructors
eihcalibration::eihcalibration(lib::configurator &_config): common::task::task(_config){
	smoothgen = NULL;
	nose = NULL;
	linear_gen = NULL;
	generator = NULL;
};

//Desctructor
eihcalibration::~eihcalibration(){

};

//methods for ECP template to redefine in concrete classes
void eihcalibration::task_initialization(void) {

    // Create an adequate robot. - depending on the ini section name.
/*    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new ecp_irp6_on_track_robot (*this);
        sr_ecp_msg->message("IRp6ot loaded");
    }
    else */if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
    	ecp_m_robot = new robot (*this);
    	sr_ecp_msg->message("IRp6p loaded");
    }

	smoothgen = new common::generator::smooth(*this, true);

	nose = new common::generator::eih_nose_run(*this, 8);
	nose->configure_pulse_check (true);

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA, "[vsp_cvfradia]", *this, sizeof(lib::sensor_image_t::sensor_union_t::chessboard_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	sr_ecp_msg->message("ECP loaded eihcalibration");

	generator = new generator::eihgenerator(*this);
	generator->sensor_m = sensor_m;
};

void eihcalibration::main_task_algorithm(void ){

	sr_ecp_msg->message("ECP eihcalibration ready");

	//Czekam, az czujnik bedzie skonfigurowany.
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	}

	smoothgen->set_absolute();

	// wczytanie pozycji poczatkowej i przejscie do niej za pomoca smooth
	if (smoothgen->load_file_with_path("../trj/nad_stolem_postument.trj")) {
	  smoothgen->Move();
	}

	// doprowadzenie chwytaka do szachownicy "wodzeniem za nos"
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == false){
//		std::cout<<"costam"<<sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found<<std::endl;
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		nose->Move();
	}
	nose->Move();


	sleep(2);

	generator->Move();

	sr_ecp_msg->message("linear_gen\n");

	//Inicjalizacja td.
	init_td(lib::XYZ_ANGLE_AXIS, 500);

	// obrot
	set_td_coordinates(0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;

	sleep(2);

	generator->Move();

	// obrot
	set_td_coordinates(0.0, 0.0, 0.0, 0.0, -0.3, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;

	sleep(2);

	generator->Move();

	// obrot
	set_td_coordinates(0.0, 0.0, 0.0, 0.0, 0.0, -0.3, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;

	sleep(2);

	generator->Move();

	//opuszczenie chwytaka o 10 cm
	set_td_coordinates(0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;

	sleep(2);

	generator->Move();

	//przesuniecie w prawo (patrzac na manipulator z przodu)
	set_td_coordinates(0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;

	sleep(2);

	generator->Move();

	//przesuniecie do operatora (patrzac na manipulator z przodu)
	set_td_coordinates(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;

	sleep(2);

	generator->Move();

/*	// zacisniecie szczek
	set_td_coordinates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;
*/
	ecp_termination_notice();
	//ecp_wait_for_stop();
};

void eihcalibration::set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
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
void eihcalibration::init_td(lib::POSE_SPECIFICATION pspec, int internode_no){
	td.arm_type=pspec;
	td.interpolation_node_no=1;
	td.internode_step_no=internode_no;	//motion time
	td.value_in_step_no=internode_no-2;			//motion time-2 ??
}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6p::task::eihcalibration(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

