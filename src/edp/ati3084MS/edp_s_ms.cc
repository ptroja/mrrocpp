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

#include <sys/iofunc.h>
#include <termios.h>

#include <boost/bind.hpp>
#include <boost/cstdint.hpp>

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


// Rejstracja procesu VSP
ATI3084_force::ATI3084_force(common::irp6s_postument_track_effector &_master) :
	force(_master), udpClient_(SENSOR_BOARD_HOST, SENSOR_BOARD_PORT), sensor_(&udpClient_), ms_nr(0),
	sendBiasReplyArrived(false), getForceReadingReplyArrived(false), FORCE_TEST_MODE(true) {
    std::cout << "ATI3084MS -> Start!" << std::endl;
	if (FORCE_TEST_MODE) {
	//	 	printf("Konstruktor VSP!\n");

		ThreadCtl(_NTO_TCTL_IO, NULL); // nadanie odpowiednich uprawnien watkowi
		// 	printf("KONTRUKTOR EDP_S POCATEK\n");

		sendBias();
	}
}

ATI3084_force::~ATI3084_force(void)
{
	if (FORCE_TEST_MODE) {
		close(uart);
	}
	if (gravity_transformation) {
		delete gravity_transformation;
	}
    std::cout << "Destruktor edp_ATI3084_force_sensor" << std::endl;
}
;

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
	if (FORCE_TEST_MODE) {
		ftxyz=getFT(uart);
		++recvd;
		if (recvd % 100 == 0) {
			std::cout << "Recvd 100 readings" << std::endl;
			std::cout << "Ft = " << ftxyz.ft[0] << " " <<  ftxyz.ft[1] << " "
			  << ftxyz.ft[2] << " " << ftxyz.ft[3] << " " << ftxyz.ft[4] << " "
			  << ftxyz.ft[5] << std::endl;
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
    //TODO: Change to permanent callback
	sensor_.SendBias(&sendBiasController, NULL, &sendBiasResponse,
	  NewFunctorCallback(boost::bind(&ATI3084_force::handleSendBiasReply, this,
	    boost::ref(sendBiasController))));
	{
		boost::unique_lock<boost::mutex> lock(sendBiasReplyArrivedMt);
		while (!sendBiasReplyArrived) {
			sendBiasReplyArrivedCv.wait(lock);
		}
		sendBiasReplyArrived = false;
	}
}

forceReadings ATI3084_force::getFT(int fd) {
	forceReadings ftxyz;
	bool received = false;

	for (int iter_counter = 1; received == false; ++iter_counter) {
		//TODO: change to permanent callback
		sensor_.GetGenGForceReading(&genForceReadingController, NULL, &genForceReading,
		  NewFunctorCallback(boost::bind(
		  &ATI3084_force::handleGetGenForceReading, this, boost::ref(genForceReadingController))));
		{
			boost::unique_lock<boost::mutex> lock(getForceReadingReplyArrivedMt);
			std::cout << "Waiting for getForceReading" << std::endl;
			while (!getForceReadingReplyArrived) {
				getForceReadingReplyArrivedCv.wait(lock);
			}
			getForceReadingReplyArrived = false;
		}
		std::cout << "Got getForceReading" << std::endl;
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
		// Success, do nothing
	}
	{
		boost::lock_guard<boost::mutex> lock(sendBiasReplyArrivedMt);
		sendBiasReplyArrived = true;
	}
	sendBiasReplyArrivedCv.notify_all();
}

void ATI3084_force::handleGetGenForceReading(ClientRpcController& controller) {
	std::cout << "handleGetGenForceReading" << std::endl;
	if (controller.expired()) {
		std::cout << "Timeout: getGenForceReading()" << std::endl;
	} else if (controller.failed()) {
		std::cout << "Failed: getGenForceReading()" << std::endl;
		std::cout << "Reason : " << controller.errorCode() << ", " <<
		  controller.errorMessage() << std::endl;
	} else {
		std::cout << "Success: getGetForceReading" << std::endl;
		// Success, do nothing
	}
	{
		boost::lock_guard<boost::mutex> lock(getForceReadingReplyArrivedMt);
		getForceReadingReplyArrived = true;
	}
	getForceReadingReplyArrivedCv.notify_all();
}

force* return_created_edp_force_sensor(common::irp6s_postument_track_effector &_master)
{
	return new ATI3084_force(_master);
}// : return_created_sensor


} // namespace sensor
} // namespace edp
} // namespace mrrocpp
