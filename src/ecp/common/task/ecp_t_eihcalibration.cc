/// \file task/ecp_t_eihcalibration.cc
/// \brief definicja zadania kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#include "ecp/common/task/ecp_t_eihcalibration.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
eihcalibration::eihcalibration(lib::configurator &_config) : task(_config)
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

    smooth_path = config.return_string_value("smooth_path");
    delay_ms = config.return_int_value("delay");
    M = config.return_int_value("M");
    A = config.return_double_value("A");
    C = config.return_double_value("C");
    D = config.return_double_value("D");
    E = config.return_double_value("E");

	smooth2gen = new generator::smooth2(*this, true);

	nose = new generator::eih_nose_run(*this, 8);
	nose->eih_nose_run::configure_pulse_check (true);

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA, "[vsp_cvfradia]", *this, sizeof(lib::sensor_image_t::sensor_union_t::chessboard_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	generator = new generator::eihgenerator(*this);
	generator->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded eihcalibration");
}

void eihcalibration::main_task_algorithm(void ){

	int i = 0, j = 0, k, l, m = 0;
	double a, b, c, d, e;
	struct timespec delay;
	delay.tv_nsec = (delay_ms % 1000) * 1000000;//delay in ms
	delay.tv_sec = (int)(delay_ms / 1000);

	sr_ecp_msg->message("ECP eihcalibration ready");

	//Czekam, az czujnik bedzie skonfigurowany.
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	}

	smooth2gen->set_absolute();

	// wczytanie pozycji poczatkowej i przejscie do niej za pomoca smooth
	smooth2gen->load_file_with_path(smooth_path.c_str());
	smooth2gen->Move();

	sr_ecp_msg->message("Data collection\n");

	// maximum velocity and acceleration of smooth2 generator
	double vv[MAX_SERVOS_NR]={0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
	double aa[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	double coordinates[MAX_SERVOS_NR]={0.0, 0.0, -1.0 * A, 0.0, 0.0, 0.0, 0.0, 0.0};
	smooth2gen->reset();
	smooth2gen->set_relative();

	//smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, -0.025, 0.0, 0.0, 0.0, 0.0, 0.0, true);
	//smooth2gen->Move();
	//nanosleep(&delay, NULL);
/*
	smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
	smooth2gen->Move();
	nanosleep(&delay, NULL);

	smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
	smooth2gen->Move();
	nanosleep(&delay, NULL);
*/
	//opusc chwytak az przestanie "widziec" szachownice
/*	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == true){
		//opuszczenie chwytaka o 2.5 cm
		smooth2gen->load_coordinates(lib::XYZ_ANGLE_AXIS, vv, aa, 0.0, 0.0, -1.0 * A, 0.0, 0.0, 0.0, 0.0, 0.0, true);
		smooth2gen->Move();
		nanosleep(&delay, NULL);
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
		generator->Move();
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
*/
	ecp_termination_notice();
	//ecp_wait_for_stop();
}

task* return_created_ecp_task(lib::configurator &_config){
	return new eihcalibration(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

