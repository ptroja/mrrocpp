#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "edp/ati3084MK/edp_s.h"

// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

ATI3084_force::ATI3084_force(common::manip_effector &_master) :
	force(_master)
{
}

void ATI3084_force::connect_to_hardware(void)
{
	if (!master.test_mode) {
		uart = open_port();
		//printf("2\n");
		tcflush(uart, TCIFLUSH);

		sendBias();
	}
}

ATI3084_force::~ATI3084_force(void)
{
	if (!master.test_mode) {
		close(uart);
	}
	if (gravity_transformation)
		delete gravity_transformation;
}

/**************************** inicjacja czujnika ****************************/
void ATI3084_force::configure_sensor(void)
{// by Y
	is_sensor_configured = true;
	//  printf("EDP Sensor configured\n");
	sr_msg->message("EDP Sensor configured");

	if (!master.test_mode) {
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
			// TODO: rewrite this code with boost::tokenizer
			lib::Xyz_Angle_Axis_vector tab;
			lib::Homog_matrix sensor_frame;
			if (master.config.exists("sensor_in_wrist")) {
				char *tmp = strdup(master.config.value <std::string> ("sensor_in_wrist").c_str());
				char* toDel = tmp;
				for (int i = 0; i < 6; i++)
					tab[i] = strtod(tmp, &tmp);
				free(toDel);
				sensor_frame = lib::Homog_matrix(tab);
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
					= new lib::ForceTrans(lib::FORCE_SENSOR_ATI3084, frame, sensor_frame, weight, pointofgravity);

		} else {
			gravity_transformation->synchro(frame);
		}
	}
}

void ATI3084_force::wait_for_event()
{
	//	sr_msg->message("wait_for_event");

	if (!master.test_mode) {

		ftxyz = getFT();

	} else {
		usleep(1000);
	}
}

/*************************** inicjacja odczytu ******************************/
void ATI3084_force::initiate_reading(void)
{
	if (!is_sensor_configured)
		throw sensor_error(lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
}

/***************************** odczyt z czujnika *****************************/
void ATI3084_force::get_reading(void)
{
	lib::Ft_vector kartez_force;

	if (master.test_mode) {
		for (int i = 0; i < 6; ++i) {
			kartez_force[i] = 0.0;
		}
		master.force_msr_upload(kartez_force);
	} else {

		lib::Ft_vector ft_table;

		for (int i = 0; i < 6; i++) {
			ft_table[i] = static_cast <double> (ftxyz.ft[i]);
		}
		/*
		 char aaa[50];
		 sprintf(aaa,"%f",ft_table[0]);
		 sr_msg->message(aaa);
		 */
		is_reading_ready = true;

		// jesli ma byc wykorzytstywana biblioteka transformacji sil
		if (master.force_tryb == 2 && gravity_transformation) {
			for (int i = 0; i < 3; i++) {
				ft_table[i] *= 10;
				ft_table[i] /= 115;
			}
			//ft_table[i]*=(10/115);
			//			for(int i=3;i<6;i++) ft_table[i]/=333;
			for (int i = 3; i < 6; i++) {
				ft_table[i] *= 5; // by Y - korekta 5/1000
				ft_table[i] /= 940; // by Y - korekta 5/1000
			}
			lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
			// lib::Homog_matrix frame(master.force_current_end_effector_frame);
			lib::Ft_vector output = gravity_transformation->getForce(ft_table, frame);
			master.force_msr_upload(output);
		}
	}
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
	const char bias = 'b';
	if (write(uart, &bias, 1) != 1) {
		// TODO: throw
		perror("write()");
	}
}

ATI3084_force::forceReadings_t ATI3084_force::getFT()
{
	const char query = 'q';
	int r;
	int current_bytes = 14;
	int iter_counter = 0; // okresla ile razy pod rzad zostala uruchomiona petla

	//	printf("bbb \n");
	uint8_t reads[14];

	//	base_cycle = ClockCycles();

	do {
		r = 1;
		iter_counter++;

		if (write(uart, &query, 1) != 1) {
			// TODO: throw
			perror("write()");
		}
		//current_cycle = ClockCycles();
		while ((current_bytes > 0) && (r != 0)) {//printf("aaa: %d\n",r);
			r = read(uart, &(reads[14 - current_bytes]), current_bytes);

			current_bytes -= r;
		}
		//	current_cycle2 = ClockCycles();

		if (r == 0) {
			if (iter_counter == 1) {
				printf("Nie otrzymano oczekiwanej ilosci znakow: %d\n", r);
				sr_msg->message(lib::NON_FATAL_ERROR, "MK Force / Torque read error - check sensor controller");
			}

			solve_transducer_controller_failure();
		} else {
			if (iter_counter > 1) {
				sr_msg->message("MK Force / Torque sensor connection reastablished");
			}
		}
	} while (r == 0);

	forceReadings_t ftxyz;

	for (int i = 0; i < 6; i++) {
		ftxyz.ft[i] = (reads[2* i ]) << 8 | reads[2* i + 1];
	}

	return ftxyz;
}

int ATI3084_force::open_port(void)
{
	int fd = open(PORT, O_RDWR);
	if (fd == -1) {
		perror("open()");
		return -1;
	}
	//printf("3\n");

	struct termios options;
	struct termios org_port_options;

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

force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI3084_force(_master);
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
