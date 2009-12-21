/// \file task/ecp_t_eih_acquisition.cc
/// \brief definicja zadania akwizycji danych potrzebnych do kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#include "application/kalibracja/ecp_t_eih_acquisition.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
eihacquisition::eihacquisition(lib::configurator &_config) : acquisition(_config)
{
    // Create an adequate robot. - depending on the ini section name.
    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
        sr_ecp_msg->message("IRp6ot loaded");
        robot = TRACK;
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot(*this);
        sr_ecp_msg->message("IRp6p loaded");
        robot = POSTUMENT;
    }

    smooth_path = config.value<std::string>("smooth_path");
    delay_ms = config.value<int>("delay");
    M = config.value<int>("M");
    A = config.value<double>("A");
    C = config.value<double>("C");
    D = config.value<double>("D");
    E = config.value<double>("E");
    calibrated = false;

	smooth2gen = new generator::smooth(*this, true);

	nose = new generator::eih_nose_run(*this, 8);
	nose->eih_nose_run::configure_pulse_check (true);

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA, "[vsp_cvfradia]", *this, sizeof(lib::sensor_image_t::sensor_union_t::chessboard_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	generator = new generator::eihgenerator(*this);
	generator->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded eihacquisition");
	ofp.number_of_measures = config.value<int>("measures_count");
	// translation vector (from robot base to tool frame) - received from MRROC
	ofp.k = gsl_vector_calloc (3*ofp.number_of_measures);
	// rotation matrix (from robot base to tool frame) - received from MRROC
	ofp.K = gsl_matrix_calloc (3*ofp.number_of_measures, 3);
	// translation vector (from chessboard base to camera frame)
	ofp.m = gsl_vector_calloc (3*ofp.number_of_measures);
	// rotation matrix (from chessboard base to camera frame)
	ofp.M = gsl_matrix_calloc (3*ofp.number_of_measures, 3);
}

void eihacquisition::main_task_algorithm(void ){

	int i = 0, j = 0, k, l, m = 0;
	double a, b, c, d, e;
	struct timespec delay;
	delay.tv_nsec = (delay_ms % 1000) * 1000000;//delay in ms
	delay.tv_sec = (int)(delay_ms / 1000);

	sr_ecp_msg->message("ECP eihacquisition ready");

	//Czekam, az czujnik bedzie skonfigurowany.
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	}

	smooth2gen->set_absolute();

	// wczytanie pozycji poczatkowej i przejscie do niej za pomoca smooth
	smooth2gen->load_file_with_path(smooth_path.c_str());
	smooth2gen->Move();

	// doprowadzenie chwytaka do szachownicy "wodzeniem za nos"
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == false){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		nose->Move();
		generator->Move();
		store_data();
	}
	nose->Move();

	sr_ecp_msg->message("Data collection\n");

	// maximum velocity and acceleration of smooth2 generator
	double vv[MAX_SERVOS_NR]={0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
	double aa[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	//double coordinates[MAX_SERVOS_NR]={0.0, 0.0, -1.0 * A, 0.0, 0.0, 0.0, 0.0, 0.0};
	smooth2gen->set_relative();

//	std::cout<<sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found<<std::endl;

	//opusc chwytak az przestanie "widziec" szachownice
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == true && !calibrated){
		//opuszczenie chwytaka o 2.5 cm
		smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, -1.0 * A, 0.0, 0.0, 0.0, 0.0, 0.0, true);
		smooth2gen->Move();
		nanosleep(&delay, NULL);
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		generator->Move();
		store_data();
		++i;
	}

	// podnies chwytak do ostatniej pozycji w ktorej wykryto szachownice
	smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, A, 0.0, 0.0, 0.0, 0.0, 0.0, true);
	smooth2gen->Move();
	nanosleep(&delay, NULL);
	--i;
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();

	// zabezpieczenie przed przekroczeniem obszaru roboczego robota
	bool flaga = true;

	// pomachaj chwytakiem zeby zrobic fajne zdjecia
	while(i >= 0 && calibrated == false)
	{
		for(l = 0; l < 6; l += 1)
		{
			c = 0.0;
			d = 0.0;
			e = 0.0;
			if(l == 0)
			{	// obrot
				c = C;
			}
			else if(l == 1)
			{	// obrot
				c = -1.0 * C;
			}
			else if(l == 2)
			{	// obrot
				d = D;
			}
			else if(l == 3)
			{	// obrot
				d = -1.0 * D;
			}
			else if(l == 4)
			{	// obrot
				e = E;
			}
			else if(l == 5)
			{	// obrot
				e = -1.0 * E;
			}

			while(((sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found) == true)
				&& calibrated == false && m < M )
			{
				smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, 0.0, e, c, d, 0.0, 0.0, true);
				smooth2gen->Move();
				nanosleep(&delay, NULL);
				generator->Move();
				store_data();
				++m;
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			}

			if(m != 0)
			{
				//powrot do poprzedniej pozycji
				smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, 0.0, -1.0 * m * e, -1.0 * m * c, -1.0 * m * d, 0.0, 0.0, true);
				smooth2gen->Move();
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
				b = A;
			}
			else if(k == 1)
			{	//przesuniecie w lewo (patrzac na manipulator z przodu)
				b = -1.0 * A;
			}
			else if(k == 2)
			{//przesuniecie do operatora (patrzac na manipulator z przodu)
				a = A;
			}
			else if(k == 3)
			{//przesuniecie od operatora (patrzac na manipulator z przodu)
				a = -1.0 * A;
			}
			else if(k == 4)
			{	//przesuniecie na skos
				a = A;
				b = A;
			}
			else if(k == 5)
			{	//przesuniecie na skos
				a = -1.0 * A;
				b = A;
			}
			else if(k == 6)
			{	//przesuniecie na skos
				a = A;
				b = -1.0 * A;
			}
			else if(k == 7)
			{	//przesuniecie na skos
				a = -1.0 * A;
				b = -1.0 * A;
			}

			while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == true
				&& calibrated == false && flaga)
			{
				smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, a, b, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
				smooth2gen->Move();
				nanosleep(&delay, NULL);
				generator->Move();
				store_data();
				++j;
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();

				for(l = 0; l < 6; l += 1)
				{
					c = 0.0;
					d = 0.0;
					e = 0.0;
					if(l == 0)
					{	// obrot
						c = C;
					}
					else if(l == 1)
					{	// obrot
						c = -1.0 * C;
					}
					else if(l == 2)
					{	// obrot
						d = D;
					}
					else if(l == 3)
					{	// obrot
						d = -1.0 * D;
					}
					else if(l == 4)
					{	// obrot
						e = E;
					}
					else if(l == 5)
					{	// obrot
						e = -1.0 * E;
					}

					// zabezpieczenie przed przekroczeniem obszaru roboczego robota
/*start2 b>0 d<0*/				if (a > 0.0 && m == 0 && c > 0 && ((i == 0 && j == 1) || ( i == 1 && j == 1) || (i == 2 && j == 2) || (i == 3 && j == 3)))
/*start1 a>0 c>0 ot i p*/					flaga = false;

					while(((sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found) == true)
						&& (calibrated == false) && m < M && flaga)
					{
						smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, 0.0, e, c, d, 0.0, 0.0, true);
						smooth2gen->Move();
						nanosleep(&delay, NULL);
						generator->Move();
						store_data();
						++m;
						sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
					}

					flaga = true;

					if(m != 0)
					{
						//powrot do poprzedniej pozycji
						smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, 0.0, -1.0 * m * e, -1.0 * m * c, -1.0 * m * d, 0.0, 0.0, true);
						smooth2gen->Move();
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
				smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, -1.0 * j * a, -1.0 * j * b, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
				smooth2gen->Move();
				j = 0;
				nanosleep(&delay, NULL);
				sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			}
		}

		// podnies chwytak o 2.5 cm
		smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, A, 0.0, 0.0, 0.0, 0.0, 0.0, true);
		smooth2gen->Move();
		nanosleep(&delay, NULL);
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		--i;
	}

	if(calibrated)
	{
		FILE *FP;
		FP = fopen("../data/eihcalibration/M.txt","w");
		gsl_matrix_fprintf (FP, ofp.M, "%g");
		fclose(FP);
		FP = fopen("../data/eihcalibration/m.txt","w");
		gsl_vector_fprintf (FP, ofp.m, "%g");
		fclose(FP);
		FP = fopen("../data/eihcalibration/k.txt","w");
		gsl_vector_fprintf (FP, ofp.k, "%g");
		fclose(FP);
		FP = fopen("../data/eihcalibration/K.txt","w");
		gsl_matrix_fprintf (FP, ofp.K, "%g");
		fclose(FP);
	}

	ecp_termination_notice();
	//ecp_wait_for_stop();
}

bool eihacquisition::store_data(void )
{
	int i,j=0;
	if(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == true && !calibrated)
	{
		for(i=0; i<12; ++i)
		{
			// store translation vector received from robot
			if(i % 4 == 3)
			{
				// translation vector
				gsl_vector_set (ofp.k, 3 * generator->count + j, generator->tab[i]);
				gsl_vector_set (ofp.m, 3 * generator->count + j, sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.transformation_matrix[i]);
				++j;
			}
			// store rotation matrix received from robot
			else
			{
				// rotation matrix
				gsl_matrix_set (ofp.K, 3 * generator->count + j, i % 4, generator->tab[i]);
				gsl_matrix_set (ofp.M, 3 * generator->count + j, i % 4, sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.transformation_matrix[i]);
			}
		}
	}
	if (generator->count == (ofp.number_of_measures - 1))
		calibrated = true;

	//std::cout<<"pomiar "<<generator->count<<std::endl;
	return true;
}

task* return_created_ecp_task(lib::configurator &_config){
	return new eihacquisition(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

