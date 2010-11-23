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
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <exception>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <cctype>
#include <cerrno>
#include <iostream>
#include <cstddef>
#include <ctime>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "edp_s.h"

#include <pthread.h>

#include "base/edp/edp_e_manip.h"
// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

// Rejstracja procesu VSP
ATI6284_force::ATI6284_force(common::manip_effector &_master) :
        force(_master)
{
    printf("FT6284KB created !!! \n");

}

void ATI6284_force::connect_to_hardware(void)
{
        if (!(master.force_sensor_test_mode)) {

            device = comedi_open("/dev/comedi0");

            if(!device)
                printf("unable to open device !!! \n");
              //  throw runtime_error("Could not open device");

            if(comedi_apply_calibration (device, 0, 0, 0, 0, NULL) != 0)
                printf("unable to callibrate device \n");

            for(int i = 0; i < 6; i++)
                comedi_get_hardcal_converter(device, 0, i, 0, COMEDI_TO_PHYSICAL, &ADC_calib[i]);

	}

}

ATI6284_force::~ATI6284_force(void)
{
        if (!(master.force_sensor_test_mode)) {
		//delete recvSocket;
		//delete sendSocket;
	}
	if (gravity_transformation)
		delete gravity_transformation;
	printf("Destruktor edp_ATI6284_force_sensor\n");
}

/**************************** inicjacja czujnika ****************************/
void ATI6284_force::configure_sensor(void)
{// by Y

        pthread_t tid = pthread_self();

        printf("force thread id : %lu \n", tid);


	is_sensor_configured = true;
	//  printf("edp Sensor configured\n");
	sr_msg->message("edp Sensor configured");

        if (!(master.force_sensor_test_mode)) {

		// synchronize gravity transformation

		// polozenie kisci bez narzedzia wzgledem bazy
		lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION); // FORCE Transformation by Slawomir Bazant
		// lib::Homog_matrix frame(master.force_current_end_effector_frame); // pobranie aktualnej ramki

                wait_for_event();

		for (int i = 0; i < 6; ++i) {
                        bias_data[i] = datav[i];
		}

		if (!gravity_transformation) // nie powolano jeszcze obiektu
		{

                    lib::Xyz_Angle_Axis_vector tab;
                    lib::Homog_matrix sensor_frame;
                    if (master.config.exists("sensor_in_wrist")) {
                            char *tmp = strdup(master.config.value <std::string> ("sensor_in_wrist").c_str());
                            char* toDel = tmp;
                            for (int i = 0; i < 6; i++)
                                    tab[i] = strtod(tmp, &tmp);
                            free(toDel);
                            sensor_frame = lib::Homog_matrix(tab);

                    } else
                            sensor_frame = lib::Homog_matrix(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0.09);
                    // lib::Homog_matrix sensor_frame = lib::Homog_matrix(-1, 0, 0, 0,  0, -1, 0, 0,  0, 0, 1, 0.09);

                    double weight = master.config.value <double> ("weight");

                    double point[3];
                    char *tmp = strdup(master.config.value <std::string> ("default_mass_center_in_wrist").c_str());
                    char* toDel = tmp;
                    for (int i = 0; i < 3; i++)
                            point[i] = strtod(tmp, &tmp);
                    free(toDel);
                    // double point[3] = { master.config.value<double>("x_axis_arm"),
                    // 		master.config.value<double>("y_axis_arm"), master.config.return_double_value("z_axis_arm") };
                    lib::K_vector pointofgravity(point);
                    gravity_transformation
                                    = new lib::ForceTrans(edp::sensor::FORCE_SENSOR_ATI3084, frame, sensor_frame, weight, pointofgravity, is_right_turn_frame);

		} else {
			gravity_transformation->synchro(frame);
		}
	}
}

void ATI6284_force::wait_for_event()
{
    //!< odczekaj
    while ((wake_time.tv_nsec += COMMCYCLE_TIME_NS) > 1000000000) {
            wake_time.tv_sec += 1;
            wake_time.tv_nsec -= 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
    if(!(master.force_sensor_test_mode))
    {
        comedi_data_read(device, 0, 0,0, AREF_DIFF, &adc_data[0]);
        comedi_data_read(device, 0, 1,0, AREF_DIFF, &adc_data[1]);
        comedi_data_read(device, 0, 2,0, AREF_DIFF, &adc_data[2]);
        comedi_data_read(device, 0, 3,0, AREF_DIFF, &adc_data[3]);
        comedi_data_read(device, 0, 4,0, AREF_DIFF, &adc_data[4]);
        comedi_data_read(device, 0, 5,0, AREF_DIFF, &adc_data[5]);

        for(int i = 0; i < 6; i++)
        {
           datav[i] = comedi_to_physical(adc_data[i], &ADC_calib[i]);
        }
    }else
    {
        for(int i = 0; i < 6; i++)
        {
           datav[i] = 0.0;
        }
    }

}

/*************************** inicjacja odczytu ******************************/
void ATI6284_force::initiate_reading(void)
{
	lib::Ft_vector kartez_force;
	double force_fresh[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	short measure_report;

	if (!is_sensor_configured) {
                //throw sensor_error(lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}

	lib::Ft_vector ft_table;

        convert_data(datav, bias_data, force_fresh);

	for (int i = 0; i < 6; ++i) {
		ft_table[i] = force_fresh[i];
	}

	is_reading_ready = true;

	// jesli ma byc wykorzytstywana biblioteka transformacji sil
	if (gravity_transformation) {

		lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
		// lib::Homog_matrix frame(master.force_current_end_effector_frame);
		lib::Ft_vector output = gravity_transformation->getForce(ft_table, frame);
		master.force_msr_upload(output);

	}
}

/***************************** odczyt z czujnika *****************************/
void ATI6284_force::get_reading(void)
{
}
/*******************************************************************/
force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI6284_force(_master);
}// : return_created_sensor

/***************************** konwersja danych z danych binarnych na sile *****************************/
/* convert data with bias from hex data to force */
// int16_t result_raw[6] - voltage [V]
// int16_t bias_raw[6] - bias data [V]
// double force[6] - output data in N, N*m
void convert_data(double result_raw[6], double bias_raw[6], double force[6])
{
	int i, j;
	double result_voltage[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	for (i = 0; i < 6; ++i) {
                result_voltage[i] = (result_raw[i] - bias_raw[i]);
		force[i] = 0.0;
	}

	for (i = 0; i < 6; ++i) {
		for (j = 0; j < 6; ++j) {
			force[i] += (double) (result_voltage[j] * conversion_matrix[i][j]);
		}
		force[i] /= conversion_scale[i];
	}


}

/*****************************  *****************************/

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
