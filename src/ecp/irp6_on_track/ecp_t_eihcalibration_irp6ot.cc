#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_eihcalibration_irp6ot.h"
#include <unistd.h>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
eihcalibration::eihcalibration(lib::configurator &_config): common::task::task(_config){
	smoothgen = NULL;
	nose = NULL;
	linear_gen = NULL;
	generator = NULL;
}

//Desctructor
eihcalibration::~eihcalibration(){

}

//methods for ECP template to redefine in concrete classes
void eihcalibration::task_initialization(void) {

    // Create an adequate robot. - depending on the ini section name.
    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new robot (*this);
        sr_ecp_msg->message("IRp6ot loaded");
    }
/*    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
    	ecp_m_robot = new robot (*this);
    	sr_ecp_msg->message("IRp6p loaded");
    }
*/
	smoothgen = new common::generator::smooth(*this, true);

	nose = new common::generator::eih_nose_run(*this, 8);
	nose->configure_pulse_check (true);

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA, "[vsp_cvfradia]", *this, sizeof(lib::sensor_image_t::sensor_union_t::chessboard_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	sr_ecp_msg->message("ECP loaded eihcalibration");

	generator = new common::generator::eihgenerator(*this);
	generator->sensor_m = sensor_m;
}

void eihcalibration::main_task_algorithm(void ){

	sr_ecp_msg->message("ECP eihcalibration ready");

	//Czekam, az czujnik bedzie skonfigurowany.
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	}

	smoothgen->set_absolute();

	// wczytanie pozycji poczatkowej i przejscie do niej za pomoca smooth
	if (smoothgen->load_file_with_path("../trj/eihcalibration/eih_calibration_start1_track.trj")) {
	  smoothgen->Move();
	}

	// doprowadzenie chwytaka do szachownicy "wodzeniem za nos"
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == false){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		nose->Move();
		generator->Move();
	}
	nose->Move();

	sr_ecp_msg->message("linear_gen\n");

	//Inicjalizacja td.
	init_td(lib::XYZ_ANGLE_AXIS, 500);

	int i = 0, j = 0, k, l, m = 0;
	double a, b, c, d, e;
	struct timespec delay;
	delay.tv_nsec = 400000000;//400ms
	delay.tv_sec = 0;

	set_td_coordinates(0.0, 0.0, -0.025, 0.0, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	//opusc chwytak az przestanie "widziec" szachownice
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == true){
		//opuszczenie chwytaka o 2.5 cm
		linear_gen->Move();
		nanosleep(&delay, NULL);
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		generator->Move();
		++i;
	}
	delete linear_gen;

	// podnies chwytak do ostatniej pozycji w ktorej wykryto szachownice
	set_td_coordinates(0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0);
	linear_gen=new common::generator::linear(*this, td, 1);
	linear_gen->Move();
	delete linear_gen;
	nanosleep(&delay, NULL);
	--i;
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();

	// zabezpieczenie przed przekroczeniem obszaru roboczego robota
	bool flaga = true;

	// pomachaj chwytakiem zeby zrobic fajne zdjecia
	while(i >= 0 && sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.calibrated == false)
	{
		if (i % 2 == 1)
			l = 1;
		else
			l = 0;

		for(; l < 6; l += 2)
		{
			c = 0.0;
			d = 0.0;
			e = 0.0;
			if(l == 0)
			{	// obrot
//				c = 0.15;
//				c = 0.02;
				c = 0.06;
			}
			else if(l == 1)
			{	// obrot
//				c = -0.15;
//				c = -0.02;
				c = -0.06;
			}
			else if(l == 2)
			{	// obrot
//				d = -0.14;
//				d = -0.03;
				d = -0.05;
			}
			else if(l == 3)
			{	// obrot
//				d = 0.14;
//				d = 0.03;
				d = -0.05;
			}
			else if(l == 4)
			{	// obrot
//				e = 0.2;
//				e = 0.05;
				e = 0.06;
			}
			else if(l == 5)
			{	// obrot
//				e = -0.2;
//				e = -0.05;
				e = -0.06;
			}

			set_td_coordinates(0.0, 0.0, 0.0, e, c, d, 0.0);
			linear_gen=new common::generator::linear(*this, td, 1);

			while(((sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found) == true)
				&& ((sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.calibrated) == false) && m < 1 )
			{
				linear_gen->Move();
				nanosleep(&delay, NULL);
				generator->Move();
				++m;
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			}

			delete linear_gen;

			if(m != 0)
			{
				//powrot do poprzedniej pozycji
				set_td_coordinates(0.0, 0.0, 0.0, -1 * m * e, -1 * m * c, -1 * m * d, 0.0);
				linear_gen=new common::generator::linear(*this, td, 1);
				linear_gen->Move();
				delete linear_gen;
				m = 0;
				nanosleep(&delay, NULL);
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			}
		}

		for(k = 0; k < 8; ++k)
		{
			a = 0.0;
			b = 0.0;
			if(k == 0)
			{	//przesuniecie w prawo (patrzac na manipulator z przodu)
				b = 0.025;
			}
			else if(k == 1)
			{	//przesuniecie w lewo (patrzac na manipulator z przodu)
				b = -0.025;
			}
			else if(k == 2)
			{//przesuniecie do operatora (patrzac na manipulator z przodu)
				a = 0.025;
			}
			else if(k == 3)
			{//przesuniecie od operatora (patrzac na manipulator z przodu)
				a = -0.025;
			}
			else if(k == 4)
			{	//przesuniecie na skos
				a = 0.025;
				b = 0.025;
			}
			else if(k == 5)
			{	//przesuniecie na skos
				a = -0.025;
				b = 0.025;
			}
			else if(k == 6)
			{	//przesuniecie na skos
				a = 0.025;
				b = -0.025;
			}
			else if(k == 7)
			{	//przesuniecie na skos
				a = -0.025;
				b = -0.025;
			}

			while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == true
				&& sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.calibrated == false && flaga)
			{
				set_td_coordinates(a, b, 0.0, 0.0, 0.0, 0.0, 0.0);
				linear_gen=new common::generator::linear(*this, td, 1);
				linear_gen->Move();
				nanosleep(&delay, NULL);
				generator->Move();
				++j;
				delete linear_gen;
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();

				if (a < 0.0)
					l = 1;
				else
					l = 0;

				for(; l < 6; l += 2)
				{

					c = 0.0;
					d = 0.0;
					e = 0.0;
					if(l == 0)
					{	// obrot
//						c = 0.15;
//						c = 0.03;
						c = 0.06;
					}
					else if(l == 1)
					{	// obrot
//						c = -0.15;
//						c = -0.03;
						c = -0.06;
					}
					else if(l == 2)
					{	// obrot
//						d = -0.14;
//						d = -0.04;
						d = -0.05;
					}
					else if(l == 3)
					{	// obrot
//						d = 0.14;
//						d = 0.04;
						d = -0.05;
					}
					else if(l == 4)
					{	// obrot
//						e = 0.2;
//						e = 0.1;
						e = 0.07;
					}
					else if(l == 5)
					{	// obrot
//						e = -0.2;
//						e = -0.1;
						e = -0.07;
					}

					set_td_coordinates(0.0, 0.0, 0.0, e, c, d, 0.0);
					linear_gen=new common::generator::linear(*this, td, 1);

					// zabezpieczenie przed przekroczeniem obszaru roboczego robota
/*start2 b>0 d<0*/					if (a > 0.0 && m == 0 && c > 0 && ((i == 0 && j == 1) || ( i == 1 && j == 1) || (i == 2 && j == 2) || (i == 3 && j == 3)))
/*start1 a>0 c>0*/						flaga = false;

					while(((sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found) == true)
						&& ((sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.calibrated) == false) && m < 1 && flaga)
					{
						linear_gen->Move();
						nanosleep(&delay, NULL);
						generator->Move();
						++m;
						sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
					}

					delete linear_gen;

					flaga = true;

					if(m != 0)
					{
						//powrot do poprzedniej pozycji
						set_td_coordinates(0.0, 0.0, 0.0, -1 * m * e, -1 * m * c, -1 * m * d, 0.0);
						linear_gen=new common::generator::linear(*this, td, 1);
						linear_gen->Move();
						delete linear_gen;
						m = 0;
						nanosleep(&delay, NULL);
						sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
					}
				}
				// zabezpieczenie przed przekroczeniem obszaru roboczego robota
/*start2 b>0*/				if(a > 0.0 && ((i == 1 && j == 1) || (i == 2 && j == 2) || (i == 3 && j == 3) || (i == 0 && j == 1)))
/*start1 a>0*/					flaga = false;
			}

			flaga = true;

			if(j != 0)
			{
				//powrot do poprzedniej pozycji
				set_td_coordinates(-1.0 * j * a, -1.0 * j * b, 0.0, 0.0, 0.0, 0.0, 0.0);
				linear_gen=new common::generator::linear(*this, td, 1);
				linear_gen->Move();
				delete linear_gen;
				j = 0;
				nanosleep(&delay, NULL);
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			}
		}

		// podnies chwytak o 2.5 cm
		set_td_coordinates(0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0);
		linear_gen=new common::generator::linear(*this, td, 1);
		linear_gen->Move();
		delete linear_gen;
		nanosleep(&delay, NULL);
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		--i;
	}

	ecp_termination_notice();
	//ecp_wait_for_stop();
}

void eihcalibration::set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	td.coordinate_delta[0] = cor0; // przyrost wspolrzednej X
	td.coordinate_delta[1] = cor1; // przyrost wspolrzednej Y
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
	return new irp6ot::task::eihcalibration(_config);
}

} // namespace task
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

