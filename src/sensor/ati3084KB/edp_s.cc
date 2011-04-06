// -------------------------------------------------------------------------
//                            edp_s.cc 		dla QNX6.3.0
//
//            Virtual Sensor Process (lib::VSP) - methods for Schunk force/torgue sensor
// Metody klasy VSP
//
// Ostatnia modyfikacja: styczen 2010
// Autor: labi (Kamil Tarkowski)
// Autor: Yoyek (Tomek Winiarski)
// na podstawie szablonu vsp Tomka Kornuty i programu obslugi czujnika Artura Zarzyckiego
// -------------------------------------------------------------------------

#include <cstdio>
#include <exception>
#include <ctime>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"

#include "base/lib/sr/srlib.h"
#include "edp_s.h"

#include "base/edp/edp_e_manip.h"
// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

#define MUX0 1
#define MUX1 2
#define MUX2 0

// Rejstracja procesu VSP
ATI3084_force::ATI3084_force(common::manip_effector &_master) :
	force(_master), dev_name("/dev/comedi0")
{
	printf("FT3084KB created !!! \n");
	flushall();

	conversion_matrix << -0.000022, 0.001325, -0.035134, 0.640126, 0.051951, -0.641909, 0.017570, -0.743414, -0.016234, 0.372558, -0.032329, 0.366082, -1.184654, -0.012028, -1.165485, -0.014266, -1.174821, 0.002540, 0.007847, -0.144965, 0.552931, 0.079813, -0.571950, 0.071877, -0.661215, -0.007048, 0.337836, -0.125610, 0.315335, 0.132327, -0.010556, 0.346443, -0.009666, 0.344562, -0.031572, 0.339944;

	conversion_scale << -20.4, -20.4, -20.4, -1.23, -1.23, -1.23;

	sensor_frame = lib::Homog_matrix(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0.09);
	force_sensor_name = edp::sensor::FORCE_SENSOR_ATI3084;

}

void ATI3084_force::connect_to_hardware(void)
{

	device = comedi_open(dev_name.c_str());

	if (!device)
		printf("unable to open device !!! \n");
	//  throw runtime_error("Could not open device");

	//char * file_path = comedi_get_default_calibration_path(device);
	//comedi_calibration_t* calib = comedi_parse_calibration_file(file_path);

	//if (calib)
	//	printf("unable to callibrate device \n");

	//for (int i = 0; i < 6; i++)
	//	comedi_get_softcal_converter(0, 0, 0, COMEDI_TO_PHYSICAL, calib, &ADC_calib[i]);

	//comedi_cleanup_calibration(calib);
	//free(file_path);

	maxdata = comedi_get_maxdata(device, 0, 0);
	rangetype = comedi_get_range(device, 0, 0, 0);

}

ATI3084_force::~ATI3084_force(void)
{
	if (!force_sensor_test_mode) {
		disconnect_from_hardware();
	}
	printf("Destruktor edp_ATI3084_force_sensor\n");
}

void ATI3084_force::disconnect_from_hardware(void)
{
	if (device)
		comedi_close(device);
}

/**************************** inicjacja czujnika ****************************/
void ATI3084_force::configure_particular_sensor(void)
{

	// synchronize gravity transformation

	wait_for_particular_event();

	bias_data = datav;

}

void ATI3084_force::wait_for_particular_event()
{
	//!< odczekaj
	lib::timespec_increment_ns(&wake_time, COMMCYCLE_TIME_NS);

	int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
	if (err != 0) {
		fprintf(stderr, "clock_nanosleep(): %s\n", strerror(err));
	}

	comedi_dio_write(device, 2, MUX0, 0);
	comedi_dio_write(device, 2, MUX1, 0);
	comedi_dio_write(device, 2, MUX2, 0);

	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[0]);
	////// G1

	comedi_dio_write(device, 2, MUX0, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[1]);
	////// G2

	comedi_dio_write(device, 2, MUX1, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[2]);
	////// G3

	comedi_dio_write(device, 2, MUX0, 0);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[3]);
	////// G4

	comedi_dio_write(device, 2, MUX2, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[4]);
	////// G5

	comedi_dio_write(device, 2, MUX0, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[5]);
	////// G6

	//      comedi_dio_write(device, 2, MUX1, 0);
	//      usleep(100);
	//      comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[6]);
	////// T


	for (int i = 0; i < 6; i++) {
		//datav[i] = comedi_to_physical(adc_data[i], &ADC_calib[i]);
		datav[i] = comedi_to_phys(adc_data[i], rangetype, maxdata);
	}

}

/***************************** odczyt z czujnika *****************************/
void ATI3084_force::get_particular_reading(void)
{

	convert_data(datav, bias_data, ft_table);

}
/*******************************************************************/
force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI3084_force(_master);
}// : return_created_sensor

/***************************** konwersja danych z danych binarnych na sile *****************************/
/* convert data with bias from hex data to force */
// int16_t result_raw[6] - voltage [V]
// int16_t bias_raw[6] - bias data [V]
// double force[6] - output data in N, N*m
void ATI3084_force::convert_data(const Vector6d &result_raw, const Vector6d &bias_raw, lib::Ft_vector &force) const
{
	Matrix <double, 6, 1> result_voltage;

	result_voltage = result_raw - bias_raw;

	force = conversion_matrix * result_voltage;
	force = force.cwise() * conversion_scale;
}

/*****************************  *****************************/

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
