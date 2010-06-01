/// \file task/ecp_t_eih_acquisition.cc
/// \brief definicja zadania akwizycji danych potrzebnych do kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#include "ecp_st_acq_eih.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
acq_eih::acq_eih(task &_ecp_t) :
	acquisition(_ecp_t) {
	printf("acq_eih::acq_eih() 1\n");
	fflush(stdout);
	// Create an adequate robot. - depending on the ini section name.
	if (ecp_sub_task::ecp_t.config.section_name == ECP_IRP6OT_M_SECTION) {
		ecp_sub_task::ecp_t.ecp_m_robot = new irp6ot_m::robot(_ecp_t);
		ecp_sub_task::ecp_t.sr_ecp_msg->message("IRp6ot loaded");
		robot = TRACK;
	} else if (ecp_sub_task::ecp_t.config.section_name == ECP_IRP6P_M_SECTION) {
		ecp_sub_task::ecp_t.ecp_m_robot = new irp6p_m::robot(_ecp_t);
		ecp_sub_task::ecp_t.sr_ecp_msg->message("IRp6p loaded");
		robot = POSTUMENT;
	}

	printf("acq_eih::acq_eih() 2\n");
	fflush(stdout);

	smooth_path = ecp_sub_task::ecp_t.config.value<std::string> ("smooth_path");
	delay_ms = ecp_sub_task::ecp_t.config.value<int> ("delay");
	M = ecp_sub_task::ecp_t.config.value<int> ("M");
	A = ecp_sub_task::ecp_t.config.value<double> ("A");
	C = ecp_sub_task::ecp_t.config.value<double> ("C");
	D = ecp_sub_task::ecp_t.config.value<double> ("D");
	E = ecp_sub_task::ecp_t.config.value<double> ("E");
	acc = ecp_sub_task::ecp_t.config.value<double> ("acceleration");
	vel = ecp_sub_task::ecp_t.config.value<double> ("velocity");
	calibrated = false;

	printf("acq_eih::acq_eih() 3\n");
	fflush(stdout);
	smoothgen = new generator::smooth(_ecp_t, true);
	printf("acq_eih::acq_eih() 4\n");
	fflush(stdout);

	nose = new generator::eih_nose_run(_ecp_t, 8);

	printf("acq_eih::acq_eih() 5\n");
	fflush(stdout);
	nose->eih_nose_run::configure_pulse_check(true);

	fradia = new ecp_mp::sensor::fradia_sensor<lib::empty_t, chessboard_t, eihcalibration_t>(_ecp_t.config, "[vsp_fradia_sensor]");
	ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA] = fradia;

	ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	generator = new generator::eihgenerator(_ecp_t);
	generator->sensor_m = ecp_sub_task::ecp_t.sensor_m;

	printf("acq_eih::acq_eih() 7\n");
	fflush(stdout);

	ecp_sub_task::ecp_t.sr_ecp_msg->message("ECP loaded eihacquisition");

	// TODO: UWAGA: TU JEST WIELKI BUG: pole ofp nie jest zainicjalizowane
	ofp.number_of_measures = ecp_sub_task::ecp_t.config.value<int> ("measures_count");

	// translation vector (from robot base to tool frame) - received from MRROC
	ofp.k = gsl_vector_calloc(3 * ofp.number_of_measures);

	printf("acq_eih::acq_eih() 9\n");
	fflush(stdout);
	// rotation matrix (from robot base to tool frame) - received from MRROC
	ofp.K = gsl_matrix_calloc(3 * ofp.number_of_measures, 3);

	printf("acq_eih::acq_eih() 10\n");
	fflush(stdout);
	// translation vector (from chessboard base to camera frame)
	ofp.m = gsl_vector_calloc(3 * ofp.number_of_measures);

	printf("acq_eih::acq_eih() 11\n");
	fflush(stdout);
	// rotation matrix (from chessboard base to camera frame)
	ofp.M = gsl_matrix_calloc(3 * ofp.number_of_measures, 3);

	printf("acq_eih::acq_eih() 12\n");
	fflush(stdout);
}

void acq_eih::main_task_algorithm(void) {

	int i = 0, j = 0, k, l, m = 0;
	double a, b, c, d, e;
	struct timespec delay;
	delay.tv_nsec = (delay_ms % 1000) * 1000000;//delay in ms
	delay.tv_sec = (int) (delay_ms / 1000);

	ecp_sub_task::ecp_t.sr_ecp_msg->message("ECP eihacquisition ready");

	//Czekam, az czujnik bedzie skonfigurowany.
	//ecp_mp::sensor::fradia_sensor<chessboard_t,lib::empty_t> * fradia = dynamic_cast<ecp_mp::sensor::fradia_sensor<chessboard_t,lib::empty_t> *> (ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA]);
	fradia->get_reading();
	while(fradia->get_report() == lib::VSP_SENSOR_NOT_CONFIGURED) {
		fradia->get_reading();
	}

	smoothgen->set_absolute();

	// wczytanie pozycji poczatkowej i przejscie do niej za pomoca smooth
	smoothgen->load_file_with_path(smooth_path.c_str());
	smoothgen->Move();

	// doprowadzenie chwytaka do szachownicy "wodzeniem za nos"
	/*
	while(fradia->get_reading_message().found == false){
		fradia->get_reading();
		nose->Move();
		generator->Move();
		store_data();
	}
	nose->Move();*/

	ecp_sub_task::ecp_t.sr_ecp_msg->message("Data collection\n");

	// maximum velocity and acceleration of smooth generator
	double vv[MAX_SERVOS_NR] = { vel, vel, vel, vel, vel, vel, vel, vel };
	double aa[MAX_SERVOS_NR] = { acc, acc, acc, acc, acc, acc, acc, acc };
	//double coordinates[MAX_SERVOS_NR]={0.0, 0.0, -1.0 * A, 0.0, 0.0, 0.0, 0.0, 0.0};
	smoothgen->set_relative();

	//	std::cout<<sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found<<std::endl;

	//opusc chwytak az przestanie "widziec" szachownice
	while(fradia->get_reading_message().found == true && !calibrated){
		//opuszczenie chwytaka o 2.5 cm
		smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0,
				A, 0.0, 0.0, 0.0, 0.0, 0.0, true);
		smoothgen->Move();
		nanosleep(&delay, NULL);
		ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		generator->Move();
		store_data();
		++i;

	}

	// podnies chwytak do ostatniej pozycji w ktorej wykryto szachownice
	smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, A,
			0.0, 0.0, 0.0, 0.0, 0.0, true);
	smoothgen->Move();
	nanosleep(&delay, NULL);
	--i;
	ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA]->get_reading();

	// zabezpieczenie przed przekroczeniem obszaru roboczego robota
	bool flaga = true;

	// pomachaj chwytakiem zeby zrobic fajne zdjecia
	while (i >= 0 && calibrated == false) {
		for (l = 0; l < 6; l += 1) {
			c = 0.0;
			d = 0.0;
			e = 0.0;
			if (l == 0) { // obrot
				c = C;
			} else if (l == 1) { // obrot
				c = -1.0 * C;
			} else if (l == 2) { // obrot
				d = D;
			} else if (l == 3) { // obrot
				d = -1.0 * D;
			} else if (l == 4) { // obrot
				e = E;
			} else if (l == 5) { // obrot
				e = -1.0 * E;
			}

			while(((fradia->get_reading_message().found) == true)
				&& calibrated == false && m < M )
			{
				smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, 0.0, e, c, d, 0.0, 0.0, true);
				smoothgen->Move();
				nanosleep(&delay, NULL);
				generator->Move();
				store_data();
				++m;
				ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			}

			if (m != 0) {
				//powrot do poprzedniej pozycji
				smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa,
						0.0, 0.0, 0.0, -1.0 * m * e, -1.0 * m * c,
						-1.0 * m * d, 0.0, 0.0, true);printf("acq_eih::main_task_alg() 1\n");
						fflush(stdout);
				smoothgen->Move();
				m = 0;
				nanosleep(&delay, NULL);
				fradia->get_reading();
			}
		}

		for (k = 0; k < 8; ++k) {
			a = 0.0;
			b = 0.0;
			if (k == 0) { //przesuniecie w prawo (patrzac na manipulator z przodu)
				b = A;
			} else if (k == 1) { //przesuniecie w lewo (patrzac na manipulator z przodu)
				b = -1.0 * A;
			} else if (k == 2) {//przesuniecie do operatora (patrzac na manipulator z przodu)
				a = A;
			} else if (k == 3) {//przesuniecie od operatora (patrzac na manipulator z przodu)
				a = -1.0 * A;
			} else if (k == 4) { //przesuniecie na skos
				a = A;
				b = A;
			} else if (k == 5) { //przesuniecie na skos
				a = -1.0 * A;
				b = A;
			} else if (k == 6) { //przesuniecie na skos
				a = A;
				b = -1.0 * A;
			} else if (k == 7) { //przesuniecie na skos
				a = -1.0 * A;
				b = -1.0 * A;
			}

			while(fradia->get_reading_message().found == true && calibrated == false && flaga)
			{
				smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, a, b, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
				smoothgen->Move();
				nanosleep(&delay, NULL);
				generator->Move();
				store_data();
				++j;
				fradia->get_reading();

				for (l = 0; l < 6; l += 1) {
					c = 0.0;
					d = 0.0;
					e = 0.0;
					if (l == 0) { // obrot
						c = C;
					} else if (l == 1) { // obrot
						c = -1.0 * C;
					} else if (l == 2) { // obrot
						d = D;
					} else if (l == 3) { // obrot
						d = -1.0 * D;
					} else if (l == 4) { // obrot
						e = E;
					} else if (l == 5) { // obrot
						e = -1.0 * E;
					}

					// zabezpieczenie przed przekroczeniem obszaru roboczego robota
					/*start2 b>0 d<0*/
					if (a > 0.0 && m == 0 && c > 0 && ((i == 0 && j == 1) || (i
							== 1 && j == 1) || (i == 2 && j == 2) || (i == 3
							&& j == 3)))
						/*start1 a>0 c>0 ot i p*/flaga = false;

					while(((fradia->get_reading_message().found) == true)
						&& (calibrated == false) && m < M && flaga)
					{
						smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, 0.0, e, c, d, 0.0, 0.0, true);
						smoothgen->Move();
						nanosleep(&delay, NULL);
						generator->Move();
						store_data();
						++m;
						fradia->get_reading();
					}

					flaga = true;

					if (m != 0) {
						//powrot do poprzedniej pozycjiodczyt danych do obliczen z zadanych plikow
						smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS,
								vv, aa, 0.0, 0.0, 0.0, -1.0 * m * e, -1.0 * m
										* c, -1.0 * m * d, 0.0, 0.0, true);
						smoothgen->Move();
						m = 0;
						nanosleep(&delay, NULL);
						fradia->get_reading();
					}
				}
				// zabezpieczenie przed przekroczeniem obszaru roboczego robota
				/*start2 b>0*/if (a > 0.0 && ((i == 1 && j == 1) || (i == 2
						&& j == 2) || (i == 3 && j == 3) || (i == 0 && j == 1)))
					/*start1 a>0*/flaga = false;
			}

			flaga = true;

			if (j != 0) {
				//powrot do poprzedniej pozycji
				smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa,
						-1.0 * j * a, -1.0 * j * b, 0.0, 0.0, 0.0, 0.0, 0.0,
						0.0, true);
				smoothgen->Move();
				j = 0;
				nanosleep(&delay, NULL);
				fradia->get_reading();
			}
		}

		// podnies chwytak o 2.5 cm
		smoothgen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0,
				-1.0 * A, 0.0, 0.0, 0.0, 0.0, 0.0, true);
		smoothgen->Move();
		nanosleep(&delay, NULL);
		fradia->get_reading();
		--i;
	}
	if (calibrated) {

		FILE *FP;
		FP = fopen(M_fp.c_str(), "w");
		gsl_matrix_fprintf(FP, ofp.M, "%g");
		fclose(FP);
		FP = fopen(mm_fp.c_str(), "w");
		gsl_vector_fprintf(FP, ofp.m, "%g");
		fclose(FP);
		FP = fopen(kk_fp.c_str(), "w");
		gsl_vector_fprintf(FP, ofp.k, "%g");
		fclose(FP);
		FP = fopen(K_fp.c_str(), "w");
		gsl_matrix_fprintf(FP, ofp.K, "%g");
		fclose(FP);
	}

	//ecp_termination_notice();
	//ecp_wait_for_stop();
}

bool acq_eih::store_data(void )
{
	int i,j=0;

	//ecp_mp::sensor::fradia_sensor<chessboard_t,lib::empty_t> * fradia = dynamic_cast<ecp_mp::sensor::fradia_sensor<chessboard_t,lib::empty_t> *> (ecp_sub_task::ecp_t.sensor_m[lib::SENSOR_CVFRADIA]);

	if(fradia->get_reading_message().found == true && !calibrated)
	{
		for(i=0; i<12; ++i)
		{
			// store translation vector received from robot
			if (i % 4 == 3) {
				// translation vector
				gsl_vector_set (ofp.k, 3 * generator->count + j, generator->tab[i]);
				gsl_vector_set (ofp.m, 3 * generator->count + j, fradia->get_reading_message().transformation_matrix[i]);
				++j;
			}
			// store rotation matrix received from robot
			else {
				// rotation matrix
				gsl_matrix_set (ofp.K, 3 * generator->count + j, i % 4, generator->tab[i]);
				gsl_matrix_set (ofp.M, 3 * generator->count + j, i % 4, fradia->get_reading_message().transformation_matrix[i]);
			}
		}
	}
	if (generator->count == (ofp.number_of_measures - 1))
		calibrated = true;

	//std::cout<<"pomiar "<<generator->count<<std::endl;
	return true;
}

void acq_eih::write_data(std::string _K_fp, std::string _kk_fp,
		std::string _M_fp, std::string _mm_fp, int _number_of_measures) {
	K_fp = _K_fp;
	kk_fp = _kk_fp;
	M_fp = _M_fp;
	mm_fp = _mm_fp;
	ofp.number_of_measures = _number_of_measures;
	acq_eih::main_task_algorithm();
}

//task* return_created_ecp_task(lib::configurator &_config){
//	return new eihacquisition(_config);
//}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

