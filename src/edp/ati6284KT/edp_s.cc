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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <process.h>
#include <math.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <string.h>
#include <fstream>
#include <iomanip>
#include <ctype.h>
#include <errno.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <iostream>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <sys/dispatch.h>
#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <stddef.h>
#include <sys/mman.h>
#include <time.h>
#include <fstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "edp_s.h"

#include <sys/syspage.h>
#include <sys/neutrino.h>


// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {


int sint_id;
struct sigevent sevent;

uint64_t *int_timeout;// by Y
struct sigevent tim_event;


struct timespec start[9];

static const char* interface = "en1";
static uint8_t boardMac[6] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static const char* host = "192.168.18.200";
static const uint16_t port = 55555;

static const unsigned int DESIRED_MEASUREMENT_FREQUENCY = 500;


// #pragma off(check_stack);

// #pragma on(check_stack);

// Rejstracja procesu VSP
ATI6284_force::ATI6284_force(common::manip_effector &_master) :
	force(_master),
	socket_(interface, boardMac),
	sensor_(&socket_)

{
	frame_counter = 0; //licznik wyslanych pakietow
    printf("Initialized docent sensor\n");
	//sendSocket = NULL;
	//recvSocket = NULL;

	for(int i = 0;i<6;++i){
		adc_data[i]=0;
		bias_data[i]=0;
	}

	measuring = false;
	_master.registerReaderStartedCallback(boost::bind(&ATI6284_force::startMeasurements, this));
	_master.registerReaderStoppedCallback(boost::bind(&ATI6284_force::stopMeasurements, this));

}

void ATI6284_force::startMeasurements() {
	timeUtil.reset();
	measuring = true;
}

void ATI6284_force::stopMeasurements() {
	measuring = false;
	timeUtil.dump("/tmp/measures2.txt");
}

void ATI6284_force::connect_to_hardware(void)
{
	if (!(master.test_mode)) {
		// 	printf("Konstruktor VSP!\n");

		ThreadCtl(_NTO_TCTL_IO, NULL); // nadanie odpowiednich uprawnien watkowi

        tim_event.sigev_notify = SIGEV_UNBLOCK;// by Y
        int_timeout = new (uint64_t);


		delay(100);

		//inicjalizacja
		try{
		    //sendSocket = new RawSocket("en1", TARGET_ETHERNET_ADDRESS);
		    //recvSocket = new RawSockeib -t("en1");

		} catch(std::exception & e) {
			  throw runtime_error("Could not open device");
		  }

	}

}

ATI6284_force::~ATI6284_force(void)
{
	if (!(master.test_mode)) {
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
	is_sensor_configured = true;
	//  printf("EDP Sensor configured\n");
	sr_msg->message("EDP Sensor configured");


	if (!(master.test_mode)) {


	}
	if (master.force_tryb == 2) {
		// synchronize gravity transformation
		//		printf("master.force_tryb == 2\n");
		// polozenie kisci bez narzedzia wzgledem bazy
		lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION); // FORCE Transformation by Slawomir Bazant
		// lib::Homog_matrix frame(master.force_current_end_effector_frame); // pobranie aktualnej ramki


		//send_request(frame_counter, sendSocket); //send request for data

		usleep(250); //250us


		wait_for_event();
		wait_for_event();
		wait_for_event();
		wait_for_event();

		for (int i = 0; i < 6; ++i) {
			bias_data[i]=adc_data[i];
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
				sensor_frame = lib::Homog_matrix(tab);
				free(toDel);
				// std::cout<<sensor_frame<<std::endl;
			} else
				sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);
			// lib::Homog_matrix sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);

			double weight = master.config.value <double> ("weight");

			double point[3];
			char *tmp = strdup(master.config.value <std::string> ("default_mass_center_in_wrist").c_str());
			char* toDel = tmp;
			for (int i = 0; i < 3; i++)
				point[i] = strtod(tmp, &tmp);
			free(toDel);
			// double point[3] = { master.config.value<double>("x_axis_arm"),
			//		master.config.value<double>("y_axis_arm"), master.config.return_double_value("z_axis_arm") };
			lib::K_vector pointofgravity(point);
			gravity_transformation
					= new lib::ForceTrans(lib::FORCE_SENSOR_ATI6284, frame, sensor_frame, weight, pointofgravity);

		} else {
			gravity_transformation->synchro(frame);
		}
	}
}

void ATI6284_force::wait_for_event()
{
	const boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1500);

	static int iter_counter = 0; // okresla ile razy pod rzad zostala uruchomiona ta metoda

    //TODO: Jesli test mode, dopuszczac wylaczenie sensora

    iter_counter++;
    if (iter_counter % 2000 == 0) {
    	//printf("Measures per sec = %d\n", timeUtil.mps());
    }

    timeUtil.wait(1000);
    timeUtil.probeDelayCycle();
    if (measuring) {
    	timeUtil.startMeasurement();
    }

    ForceSensor6284::AdcReadings_t readings = sensor_.getAdcReadings(timeout);
    if (measuring) {
    	timeUtil.stopMeasurement();
    }
    if (readings.timeout) {
        printf("Expired!!!\n");
    } else {
		adc_data[0] = readings.readings[0];
        adc_data[1] = readings.readings[1];
        adc_data[2] = readings.readings[2];
        adc_data[3] = readings.readings[3];
        adc_data[4] = readings.readings[4];
        adc_data[5] = readings.readings[5];
    }

}

/*************************** inicjacja odczytu ******************************/
void ATI6284_force::initiate_reading(void)
{
	lib::Ft_vector kartez_force;
    double force_fresh[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	short measure_report;

	if (!is_sensor_configured) {
		throw sensor_error(lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
    }

    lib::Ft_vector ft_table;

    convert_data(adc_data, bias_data, force_fresh);

    for (int i = 0; i < 6; ++i) {
        ft_table[i] = force_fresh[i];
    }

    is_reading_ready = true;

    // jesli ma byc wykorzytstywana biblioteka transformacji sil
    if (master.force_tryb == 2 && gravity_transformation) {


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


/*******************************************************************/
void send_request(uint64_t &counter) {
  unsigned char send_buffer[8];
  ++counter;
  memcpy((void *)(send_buffer), (void *)(&(counter)), 8);     //64bit unsigned counter
  //sock->send(send_buffer, 8);                           //64bit counter = 8 bytes

}
/***************************** konwersja danych z danych binarnych na sile *****************************/
/* convert data with bias from hex data to force */
// int16_t result_raw[6] - input data in hex
// int16_t bias_raw[6] - bias data in hex
// double force[6] - output data in N, N*m
void convert_data(int16_t result_raw[6], int16_t bias_raw[6], double force[6]) {
  int i, j;
  double result_voltage[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

  for (i = 0; i < 6; ++i) {
    result_voltage[i] =
        (double) ((result_raw[i] - bias_raw[i]) * 10.0f / 2048.0f);
    force[i]=0.0;
  }

  for (i = 0; i < 6; ++i) {
    for (j = 0; j < 6; ++j) {
      force[i] += (double) (result_voltage[j] * conversion_matrix[i][j]);
    }
    force[i] /= conversion_scale[i];
  }

/*
	printf("\n-----------------------------------------------------------------------------------------------\n");
	printf("%c\t%c\t%c\t%c\t%c\t%c\n",'X','Y','Z','X','Y','Z');
	printf("\n-----------------------------------------------------------------------------------------------\n");
	printf("Bias reading:\n");
	for (i=0;i<6;i++)
		printf("%9.6f[V] ",bias_voltage[i]);
	printf("\n\nMeasurement:\n\n");
	for (i=0;i<6;i++)
		printf("%9.6f[V] ",voltage[i]);
	printf("\n\nDifference Measurement - Bias reading :\n\n");
	for (i=0;i<6;i++)
		printf("%9.6f[V] ",voltage[i]-bias_voltage[i]);
	printf("\n\nResult (force/torque):\n\n");
	for (i=0;i<6;i++){
		if (i<3)
			printf("%9.6f[N] ",force[i]);
		else
			printf("%9.6f[N-m] ",force[i]);
	}
	printf("\n\nResult local (force/torque):\n\n");
	for (i=0;i<6;i++){
		if (i<3)
			printf("%9.6f[N] ",force_local[i]);
		else
			printf("%9.6f[N-m] ",force_local[i]);
	}
	printf("\n-----------------------------------------------------------------------------------------------\n");

*/



}
/*******************************************************************/
/* gets data from ethernet and store it in int16_t data_raw[6] */
int get_data_from_ethernet(unsigned char buffer[512],
                           int16_t data_raw[6]) {
  std::size_t recvd = 0;//sock->recv(buffer, 512);
  unsigned char *buffer_data = buffer + 8;  //8 bytes for uint64_t

  if (recvd >= 20) {            //12B - ADC data, 8B - counter

    //convert two bytes to one word. &0xFF - take only one byte, not four...
    for (int i = 0; i < 6; ++i) {
      data_raw[i] = (((buffer_data[2 * i] & 0xFF) << 8) | (buffer_data[2 * i + 1] & 0xFF));

    }
    return 0;
  } else {
    return -1;
  }
}

/*****************************  *****************************/

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
