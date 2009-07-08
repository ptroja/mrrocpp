// -------------------------------------------------------------------------
//                            edp_s.cc 		dla QNX6.3.0
//
//            Virtual Sensor Process (lib::VSP) - methods for Schunk force/torgue sensor
// Metody klasy VSP
//
// Ostatnia modyfikacja: grudzie 2004
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
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <pthread.h>
#include <inttypes.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "edp/ati3084MK/edp_s.h"

// Konfigurator
#include "lib/configurator.h"


void y_InterruptUnlock( intrspin_t* spinlock_local )
{
	InterruptUnlock(spinlock_local );
}


namespace mrrocpp {
namespace edp {
namespace sensor {




unsigned int ms_nr=0;// numer odczytu z czujnika


struct timespec start[9];


// Rejstracja procesu VSP
ATI3084_force::ATI3084_force(common::irp6s_postument_track_effector &_master) :
	force(_master)
	{
	if (!(master.test_mode)) {
	//	 	printf("Konstruktor VSP!\n");

		ThreadCtl(_NTO_TCTL_IO, NULL); // nadanie odpowiednich uprawnien watkowi
		// 	printf("KONTRUKTOR EDP_S POCATEK\n");

		uart=open_port();
		//printf("2\n");
		tcflush(uart,TCIFLUSH);


		sendBias(uart);
	}
	}


ATI3084_force::~ATI3084_force(void)
{
	if (!(master.test_mode)) {

		close(uart);
	}
	if (gravity_transformation)
		delete gravity_transformation;
	printf("Destruktor edp_ATI3084_force_sensor\n");
}
;

/**************************** inicjacja czujnika ****************************/
void ATI3084_force::configure_sensor(void)
{// by Y
	is_sensor_configured=true;
	//  printf("EDP Sensor configured\n");
	sr_msg->message("EDP Sensor configured");

	if (!(master.test_mode))
	{
		sendBias(uart);
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
					tab[i] = strtod( tmp, &tmp );
				free(toDel);
				sensor_frame = lib::Homog_matrix(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, tab[0], tab[1], tab[2], tab[3], tab[4], tab[5]);
				// std::cout<<sensor_frame<<std::endl;
			}
			else
				sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);
			// lib::Homog_matrix sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);

			double weight = master.config.return_double_value("weight");

			double point[3];
			char *tmp = strdup(master.config.return_string_value("default_mass_center_in_wrist").c_str());
			char* toDel = tmp;
			for (int i=0; i<3; i++)
				point[i] = strtod( tmp, &tmp );
			free(toDel);
			// double point[3] = { master.config.return_double_value("x_axis_arm"),
			//		master.config.return_double_value("y_axis_arm"), master.config.return_double_value("z_axis_arm") };
			lib::K_vector pointofgravity(point);
			gravity_transformation = new lib::ForceTrans(lib::FORCE_SENSOR_ATI3084, frame, sensor_frame, weight, pointofgravity);

		} else {
			gravity_transformation->synchro(frame);
		}
	}

}


void ATI3084_force::wait_for_event()
{

//	sr_msg->message("wait_for_event");

	if (!(master.test_mode))
	{

		ftxyz=getFT(uart);


	} else {
		usleep(1000);
	}
}
;

/*************************** inicjacja odczytu ******************************/
void ATI3084_force::initiate_reading(void)
{
	double kartez_force[6];
	short measure_report;

	if (!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	if (master.test_mode) {
		for (int i = 0; i < 6; ++i) {
			kartez_force[i] = 0.0;
		}
		master.force_msr_upload(kartez_force);
	} else
	{

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
	sendBias(uart);
	usleep(100);
}



void ATI3084_force::sendBias(int fd)
{
	char bias='b';
	if (write(fd, &bias, 1) <0) printf("Blad zapisu\n");
}

forceReadings ATI3084_force::getFT(int fd)
{
	char query='q';
	int i,r;
	int current_bytes=14;
	int iter_counter=0; // okresla ile razy pod rzad zostala uruchomiona ta metoda

	//	printf("bbb \n");
	uint8_t reads[14];
	forceReadings ftxyz;

	//	base_cycle = ClockCycles();



	do
	{
		r=1;
		iter_counter++;

		if (write(fd, &query, 1) <0) printf("Blad zapisu\n");
		//current_cycle = ClockCycles();
		while ((current_bytes>0)&&(r!=0))
		{//printf("aaa: %d\n",r);
			r=read(fd, &(reads[14-current_bytes]), current_bytes);

			current_bytes-=r;
		}
		//	current_cycle2 = ClockCycles();

		if (r==0) {
			if (iter_counter==1)
			{
				printf("Nie otrzymano oczekiwanej ilosci znakow: %d\n",r);
				sr_msg->message(lib::NON_FATAL_ERROR, "MK Force / Torque read error - check sensor controller");
			}

			solve_transducer_controller_failure();
		}else {
			if (iter_counter>1) {
				sr_msg->message("MK Force / Torque sensor connection reastablished");
			}
		}
	} while (r==0);



	for (i=0;i<6;i++) {
		ftxyz.ft[i]=(reads[2*i])<<8 | reads[2*i+1];
	}


	return ftxyz;
}

int ATI3084_force::open_port(void)
{
	int fd;
	struct termios options;
	struct termios org_port_options;
	fd = open(PORT, O_RDWR);
	if(fd == -1)
	{
		printf("Can not open the port");
		return -1;
	}
	//printf("3\n");
	tcgetattr(fd, &org_port_options);
	options = org_port_options;

	cfsetispeed(&options, SPEED);
	cfsetospeed(&options, SPEED);

	options.c_cflag |= CLOCAL; // Local line - Do not change the owner
	options.c_cflag |= CREAD; // Enable receiver
	options.c_cflag &= ~PARENB; // parity - enabled
	options.c_cflag &= ~PARODD; // Odd parity - even
	options.c_cflag &= ~CSTOPB; // stop bit - 1
	options.c_cflag &= ~CSIZE; // bit mask for data bits
	options.c_cflag |= CS8; // data bits - 8
	options.c_lflag &= ~ECHO; // Echo the Terminal screen - off
	options.c_lflag &= ~ICANON; // Canonical Input else raw - disable
	options.c_iflag &= ~INPCK; // Parity Check - disable
	options.c_iflag &= ~IGNBRK; // Parity Errors - Ignore
	options.c_iflag &= ~PARMRK; // Mark Parity Errors - disable
	options.c_iflag &= ~ISTRIP; // Strip Parity bits - disable
	options.c_iflag &= ~IXON; // Software outgoing flow control - disable
	options.c_iflag &= ~IXOFF; // Software incoming flow control - disable
	options.c_oflag &= ~OPOST; // Post process output (not set = raw output)
	options.c_cc[VTIME] = 5;
	options.c_cc[VMIN] = 0;
	tcsetattr(fd, TCSADRAIN, &options);
	fcntl(fd, F_SETFL, 0);
	//	printf("4\n");
	return fd;
}



force* return_created_edp_force_sensor(common::irp6s_postument_track_effector &_master)
{
	return new ATI3084_force(_master);
}// : return_created_sensor

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
