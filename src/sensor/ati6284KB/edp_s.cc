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

ATI6284_force::ATI6284_force(common::manip_effector &_master) :
		force(_master), dev_name("/dev/comedi1")
{
	printf("FT6284KB created !!! \n");

	conversion_matrix << -0.40709, -0.27318, 0.34868, -33.58156, -0.32609, 33.54162, 0.35472, 38.22730, -0.41173, -19.49156, 0.49550, -19.15271, 18.72635, -0.59676, 19.27843, -0.56931, 18.69352, -0.67633, -0.40836, -0.95908, -33.37957, 1.38537, 32.52522, -0.51156, 37.13715, -1.02875, -20.00474, -0.27959, -19.34135, 1.42577, -0.15775, -18.16831, -0.00133, -18.78961, 0.31895, -18.38586;

	conversion_scale << 4.5511972116989, 4.5511972116989, 1.41244051397552, 84.8843245576086, 84.8843245576086, 80.9472037525247;

	sensor_frame = lib::Homog_matrix(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0.09);
	force_sensor_name = edp::sensor::FORCE_SENSOR_ATI6284;

}

void ATI6284_force::connect_to_hardware(void)
{

	device = comedi_open(dev_name.c_str());

	if (!device) {
		throw std::runtime_error("Could not open device");
	}

	if (comedi_apply_calibration(device, 0, 0, 0, 0, NULL) != 0)
		printf("unable to callibrate device \n");

	for (int i = 0; i < 6; i++)
		comedi_get_hardcal_converter(device, 0, i, 0, COMEDI_TO_PHYSICAL, &ADC_calib[i]);

}

ATI6284_force::~ATI6284_force(void)
{
	if (!force_sensor_test_mode) {
		disconnect_from_hardware();
	}
	printf("Destruktor edp_ATI6284_force_sensor\n");
}

void ATI6284_force::disconnect_from_hardware(void)
{
	if (device)
		comedi_close(device);
}

void ATI6284_force::configure_particular_sensor(void)
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

void ATI6284_force::wait_for_particular_event()
{
	//!< odczekaj
	lib::timespec_increment_ns(&wake_time, COMMCYCLE_TIME_NS);

	int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
	if (err != 0) {
		fprintf(stderr, "clock_nanosleep(): %s\n", strerror(err));
	}

	for (int i = 0; i < 6; i++) {
		comedi_data_read(device, 0, i, 0, AREF_DIFF, &adc_data[i]);
	}

	for (int i = 0; i < 6; i++) {
		datav[i] = comedi_to_physical(adc_data[i], &ADC_calib[i]);
	}

}

void ATI6284_force::get_particular_reading(void)
{

	convert_data(datav, bias_data, ft_table);

}

force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI6284_force(_master);
}

void ATI6284_force::convert_data(const Vector6d &result_raw, const Vector6d &bias_raw, lib::Ft_vector &force) const
{
	Matrix <double, 6, 1> result_voltage;

	result_voltage = result_raw - bias_raw;

	force = conversion_matrix * result_voltage;
	force = force.cwise() / conversion_scale;
}

/*****************************  *****************************/

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

