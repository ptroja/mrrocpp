/*!
 * @file
 * @brief File containing methods of the ATI3084 Froce/Torque sensor class.
 *
 * @author Konrad Banachowicz
 *
 */

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

#define MUX0 0
#define MUX1 1
#define MUX2 2

#define DOSD 3

ATI3084_force::ATI3084_force(common::manip_effector &_master) :
		force(_master), dev_name("/dev/comedi0")
{
	printf("FT3084KB created !!! \n");
	flushall();

	// Initialize conversion matrix
	conversion_matrix << -0.000022, 0.001325, -0.035134, 0.640126, 0.051951, -0.641909, 0.017570, -0.743414, -0.016234, 0.372558, -0.032329, 0.366082, -1.184654, -0.012028, -1.165485, -0.014266, -1.174821, 0.002540, 0.007847, -0.144965, 0.552931, 0.079813, -0.571950, 0.071877, -0.661215, -0.007048, 0.337836, -0.125610, 0.315335, 0.132327, -0.010556, 0.346443, -0.009666, 0.344562, -0.031572, 0.339944;

	conversion_scale << -20.4, -20.4, -20.4, -1.23, -1.23, -1.23;

	sensor_frame = lib::Homog_matrix(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0.09);
	force_sensor_name = edp::sensor::FORCE_SENSOR_ATI3084;

}

void ATI3084_force::connect_to_hardware(void)
{

	device = comedi_open(dev_name.c_str());

	if (!device) {

		throw std::runtime_error("Could not open device");
	}

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

void ATI3084_force::configure_particular_sensor(void)
{
	// by Y
	// synchronize gravity transformation with average based filtration

	for (int l = 0; l < 6; l++) {
		bias_data[l] = 0.0;
	}

	for (int i = 0; i < BIAS_VECTOR_LENGTH; i++) {
		wait_for_particular_event();

		for (int l = 0; l < 6; l++) {
			bias_data[l] += datav[l] / ((double) BIAS_VECTOR_LENGTH);
		}

	}

}

void ATI3084_force::wait_for_particular_event()
{
	//!< odczekaj
	lib::timespec_increment_ns(&wake_time, COMMCYCLE_TIME_NS);

	int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
	if (err != 0) {
		fprintf(stderr, "clock_nanosleep(): %s\n", strerror(err));
	}

	comedi_dio_write(device, DOSD, MUX0, 0);
	comedi_dio_write(device, DOSD, MUX1, 0);
	comedi_dio_write(device, DOSD, MUX2, 0);

	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[0]);
	////// G1

	comedi_dio_write(device, DOSD, MUX0, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[1]);
	////// G2

	comedi_dio_write(device, DOSD, MUX1, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[2]);
	////// G3

	comedi_dio_write(device, DOSD, MUX0, 0);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[3]);
	////// G4

	comedi_dio_write(device, DOSD, MUX2, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[4]);
	////// G5

	comedi_dio_write(device, DOSD, MUX0, 1);
	usleep(100);
	comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[5]);
	////// G6

	//      comedi_dio_write(device, 2, MUX1, 0);
	//      usleep(100);
	//      comedi_data_read(device, 0, 0, 0, AREF_DIFF, &adc_data[6]);
	////// T

	for (int i = 0; i < 6; i++) {
		datav[i] = comedi_to_phys(adc_data[i], rangetype, maxdata);
	}

}

void ATI3084_force::get_particular_reading(void)
{

	convert_data(datav, bias_data, ft_table);

}

force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI3084_force(_master);
}

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

