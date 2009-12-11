// -------------------------------------------------------------------------
//                            edp_s.cc 		dla QNX6.3.0
//
//            Virtual Sensor Process (lib::VSP) - methods for Schunk force/torque sensor
// Metody klasy VSP
//
// Ostatnia modyfikacja: grudzie 2004
// Autor: Yoyek (Tomek Winiarski)
// na podstawie szablonu vsp Tomka Kornuty i programu obslugi czujnika Artura Zarzyckiego
// -------------------------------------------------------------------------

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <sys/iofunc.h>
#include <termios.h>
#include <sys/neutrino.h>
#include <inttypes.h>

#include <boost/bind.hpp>
#include <boost/cstdint.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <kiper/Log.hpp>
#include <kiper/FunctorCallback.hpp>
#include <kiper/RpcClient.hpp>
#include <kiper/ClientRpcController.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#include <edp/ati3084MS/edp_s_ms.h>

#include "lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

using namespace kiper;

//const char* ATI3084_force::SENSOR_BOARD_HOST = "192.168.18.200";
const char* ATI3084_force::SENSOR_BOARD_HOST = "192.160.10.20";
const char* ATI3084_force::SENSOR_DEVICE_NAME = "en0";
uint8_t ATI3084_force::SENSOR_BOARD_MAC[6] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Rejstracja procesu VSP
ATI3084_force::ATI3084_force(common::irp6s_postument_track_effector &_master) :
  //force(_master), rpcClient_(SENSOR_DEVICE_NAME, SENSOR_BOARD_MAC), sensor_(&rpcClient_), ms_nr(0),
  force(_master), rpcClient_(SENSOR_BOARD_HOST, SENSOR_BOARD_PORT), sensor_(&rpcClient_), ms_nr(0),
  sendBiasClosure_(NewPermanentFunctorCallback(boost::bind(&ATI3084_force::handleSendBiasReply, this,
  boost::ref(sendBiasController)))),
  getForceReadingClosure_(NewPermanentFunctorCallback(boost::bind(&ATI3084_force::handleGetGenForceReading, this,
  boost::ref(getForceReadingController)))),
  FORCE_TEST_MODE(true) {}

void ATI3084_force::connect_to_hardware (void)
{
	getForceReadingController.setTimeout(boost::posix_time::milliseconds(1));
	sendBiasController.setTimeout(boost::posix_time::milliseconds(1));
    std::cout << "ATI3084MS -> Start!" << std::endl;
	if (FORCE_TEST_MODE) {
	//	 	printf("Konstruktor VSP!\n");

		ThreadCtl(_NTO_TCTL_IO, NULL); // nadanie odpowiednich uprawnien watkowi
		// 	printf("KONTRUKTOR EDP_S POCATEK\n");

		sendBias();
	}
	master.registerReaderStartedCallback(boost::bind(&ATI3084_force::onReaderStarted, this));
	master.registerReaderStoppedCallback(boost::bind(&ATI3084_force::onReaderStopped, this));

}



ATI3084_force::~ATI3084_force(void)
{
	if (gravity_transformation) {
		delete gravity_transformation;
	}
    std::cout << "Destruktor edp_ATI3084_force_sensor" << std::endl;

    // Dump readings

    boost::posix_time::ptime currentTime(boost::posix_time::second_clock::local_time());
    std::string fileName = to_iso_string(currentTime);
    std::cout << "File name = " << fileName << std::endl;
    for (unsigned int i = 0; i < 10; ++i) {

    }
}


bool ATI3084_force::sensorWorks() {
	for (unsigned int i = 0; i < 3; ++i) {
		getForceReadingController.reset();
		sensor_.GetGenGForceReading(&getForceReadingController, NULL, &genForceReading,
		  getForceReadingClosure_.get());
		if (getForceReadingController.expired() || getForceReadingController.failed()) {
			std::cerr << "Sensor reading failed, retrying..." << std::endl;
		} else {
			return true;
		}
	}
	return false;
}

/**************************** inicjacja czujnika ****************************/
void ATI3084_force::configure_sensor(void)
{// by Y
	is_sensor_configured=true;
	//  printf("EDP Sensor configured\n");
	sr_msg->message("EDP Sensor configured");

	if (FORCE_TEST_MODE)
	{
		sendBias();
	}

	if (master.force_tryb == 2) {
		// synchronize gravity transformation
		//		printf("master.force_tryb == 2\n");
		// polozenie kisci bez narzedzia wzgledem bazy
		lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION); // FORCE Transformation by Slawomir Bazant
		// lib::Homog_matrix frame(master.force_current_end_effector_frame); // pobranie aktualnej ramki
		if (!gravity_transformation) // nie powolano jeszcze obiektu
		{
			// polozenie czujnika wzgledem kisci (bez narzedzia)
			//lib::frame_tab sensor_rot = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 0}};
			// polozenie czujnika wzgledem  koncowki lancucha kinematycznego
			// lib::Homog_matrix sensor_frame = lib::Homog_matrix(0, -1, 0,		1, 0, 0,	0, 0, 1,	0, 0, 0.09);

			double tab[6];
			lib::Homog_matrix sensor_frame;
			if (master.config.exists("sensor_in_wrist"))
			{
				char *tmp = strdup(master.config.return_string_value("sensor_in_wrist").c_str());
				char* toDel = tmp;
				for (int i=0; i<6; i++)
					tab[i] = std::strtod( tmp, &tmp );
				sensor_frame = lib::Homog_matrix(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, tab[0], tab[1], tab[2], tab[3], tab[4], tab[5]);
				// std::cout<<sensor_frame<<std::endl;
				free(toDel);
			}
			else
				sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);
			// lib::Homog_matrix sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);

			double weight = master.config.return_double_value("weight");

			double point[3];
			std::string lol = master.config.return_string_value("default_mass_center_in_wrist");
			char* tmp = strdup(lol.c_str());
			char* toDel = tmp;
			for (int i=0; i<3; i++)
				point[i] = strtod( tmp, &tmp );
			// double point[3] = { master.config.return_double_value("x_axis_arm"),
			//		master.config.return_double_value("y_axis_arm"), master.config.return_double_value("z_axis_arm") };
			lib::K_vector pointofgravity(point);
			gravity_transformation = new lib::ForceTrans(lib::FORCE_SENSOR_ATI3084, frame, sensor_frame, weight, pointofgravity);
			free(toDel);
		} else {
			gravity_transformation->synchro(frame);
		}
	}

}


void ATI3084_force::wait_for_event() {
	static unsigned int recvd = 0;
	usleep(300);

	static unsigned int nReads = 0;
	static const int STORED_READS = 1048576;
	static uint16_t reads[STORED_READS];
	static unsigned int iReads = 0;
	static uint64_t totalTime = 0;
	static unsigned int CPU_CLOCK = 500000000;

	if (FORCE_TEST_MODE) {
		uint64_t cycles = ClockCycles();
		ftxyz=getFT(uart);
		uint64_t elapsedCycles = ClockCycles() - cycles;
		uint16_t timeMicroSec = (uint16_t) (((double) elapsedCycles) / (CPU_CLOCK/1000000));
		totalTime += timeMicroSec;
		if (iReads == STORED_READS) {
			iReads = 0;
		}
		reads[iReads] = timeMicroSec;
		++recvd;
		++iReads;
		if (iReads % 2000 == 0) {
			std::cout << "Mean time = " << double(totalTime) / 2000 << std::endl;
			totalTime = 0;
		}
	} else {
		usleep(1000);
	}
}

/*************************** inicjacja odczytu ******************************/
void ATI3084_force::initiate_reading(void)
{
	double kartez_force[6];
	short measure_report;

	if (!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	if (!FORCE_TEST_MODE) {
		for (int i = 0; i < 6; ++i) {
			kartez_force[i] = 0.0;
		}
		master.force_msr_upload(kartez_force);
	} else {
		double ft_table[6];
		for (int i=0;i<6;i++)
		{
			ft_table[i] = static_cast<double>(ftxyz.ft[i]);
		}
/*
		char aaa[50];

		sprintf(aaa,"%f",ft_table[0]);



			sr_msg->message(aaa);

*/
		is_reading_ready=true;

		// jesli ma byc wykorzytstywana biblioteka transformacji sil
		if (master.force_tryb == 2 && gravity_transformation)
		{
			for(int i=0;i<3;i++)
			{
				ft_table[i]*=10;
			ft_table[i]/=115;
			}
			//ft_table[i]*=(10/115);
			//			for(int i=3;i<6;i++) ft_table[i]/=333;
			for(int i=3;i<6;i++)
			{
				ft_table[i]*=5; // by Y - korekta 5/1000
				ft_table[i]/=940; // by Y - korekta 5/1000
			}
			lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
			// lib::Homog_matrix frame(master.force_current_end_effector_frame);
			double* output = gravity_transformation->getForce (ft_table, frame);
			master.force_msr_upload(output);

			delete[] output;

		}
	}

}

/***************************** odczyt z czujnika *****************************/
void ATI3084_force::get_reading(void)
{
}



/********************** zakonczenie dzialania czujnika *************************/
void ATI3084_force::terminate(void)
{
	//	printf("VSP terminate\n");
}



// metoda na wypadek skasowanie pamiecia nvram
// uwaga sterownik czujnika wysyla komunikat po zlaczu szeregowym zaraz po jego wlaczeniu

void ATI3084_force::solve_transducer_controller_failure(void)
{
	usleep(10);
	sendBias();
	usleep(100);
}



void ATI3084_force::sendBias()
{
	sendBiasController.reset();
	sensor_.SendBias(&sendBiasController, NULL, &sendBiasResponse,
	  sendBiasClosure_.get());
}

forceReadings ATI3084_force::getFT(int fd) {
	forceReadings ftxyz;
	bool received = false;

	for (int iter_counter = 1; received == false; ++iter_counter) {
		//TODO: change to permanent callback
		getForceReadingController.reset();
		sensor_.GetGenGForceReading(&getForceReadingController, NULL, &genForceReading,
		  getForceReadingClosure_.get());
		received = true;
		if (!received) {
			if (iter_counter==1) {
                std::cout << "Timeout while getting force readings: " << r << std::endl;
				sr_msg->message(lib::NON_FATAL_ERROR, "MK Force / Torque read error - check sensor controller");
			}

			solve_transducer_controller_failure();
		} else {
			if (iter_counter>1) {
				sr_msg->message("MK Force / Torque sensor connection reastablished");
			}
		}
	}


	ftxyz.ft[0] = genForceReading.fx();
	ftxyz.ft[1] = genForceReading.fy();
	ftxyz.ft[2] = genForceReading.fz();
	ftxyz.ft[3] = genForceReading.tx();
	ftxyz.ft[4] = genForceReading.ty();
	ftxyz.ft[5] = genForceReading.tz();
	return ftxyz;
}

void ATI3084_force::handleSendBiasReply(ClientRpcController& controller) {
	if (controller.expired()) {
		std::cout << "Timeout: sendBias()" << std::endl;
	} else if (controller.failed()) {
		std::cout << "Failed: sendBias()" << std::endl;
		std::cout << "Reason : " << controller.errorCode() << ", " <<
		  controller.errorMessage() << std::endl;
	} else {
	}
}

void ATI3084_force::handleGetGenForceReading(ClientRpcController& controller) {
	static int iter = 0;
	if (controller.expired()) {
		std::cout << "Timeout: getGenForceReading()" << std::endl;
	} else if (controller.failed()) {
		std::cout << "Failed: getGenForceReading()" << std::endl;
		std::cout << "Reason : " << controller.errorCode() << ", " <<
		  controller.errorMessage() << std::endl;
	} else {
		++iter;
		if (iter % 1000 == 0) {
			std::cout << genForceReading.fx() << " " <<

				genForceReading.fy() << " " <<
				genForceReading.fz() << " " <<
				genForceReading.tx() << " " <<
				genForceReading.ty() << " " <<
				genForceReading.tz() << " " << std::endl;
		}
	}
}

force* return_created_edp_force_sensor(common::irp6s_postument_track_effector &_master)
{
	return new ATI3084_force(_master);
}// : return_created_sensor

void ATI3084_force::onReaderStarted() {
	std::cout << "DOCENT: reader started!" << std::endl;
}

void ATI3084_force::onReaderStopped() {
	std::cout << "DOCENT: reader stopped!" << std::endl;
}


} // namespace sensor
} // namespace edp
} // namespace mrrocpp
