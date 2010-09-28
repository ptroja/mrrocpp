//#include "robot/sarkofag/edp_e_sarkofag.h"
#include "robot/hi_moxa/hi_moxa.h"

//#include "hi_sarkofag.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

#include <exception>
#include <stdexcept>
#include <cstring>
#include <iostream>

#include "base/edp/edp_e_motor_driven.h"

namespace mrrocpp {
namespace edp {
namespace hi_moxa {

HI_moxa::HI_moxa(common::motor_driven_effector &_master, int first_drive_n, int last_drive_n) :
	common::HardwareInterface(_master), first_drive_number(first_drive_n), last_drive_number(last_drive_n)
{
#ifdef T_INFO_FUNC
	std::cout << "[func] Hi, Moxa!" << std::endl;
#endif
}

HI_moxa::~HI_moxa()
{
#ifdef T_INFO_FUNC
	std::cout << "[func] Bye, Moxa!" << std::endl;
#endif
	for (unsigned int i = first_drive_number; i <= last_drive_number; i++) {
		if (fd[i] > 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close(fd[i]);
		}
	}
}

void HI_moxa::init()
{

	std::string port = PORT;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::init()" << std::endl;
#endif

	// inicjalizacja zmiennych
	for (unsigned int i = first_drive_number; i <= last_drive_number; i++) {
		servo_data[i].first_hardware_read = true;
		servo_data[i].command_params = 0;
	}

	fd_max = 0;
	for (unsigned int i = first_drive_number; i <= last_drive_number; i++) {
		std::cout << "[info] opening port : " << (port + (char) (i + INIT_PORT_CHAR)).c_str();
		fd[i] = open((port + (char) (i + INIT_PORT_CHAR)).c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd[i] < 0) {
			//	throw(std::runtime_error("unable to open device!!!"));
			std::cout << std::endl << "[error] fd == " << (int) fd[i] << std::endl;
		} else {
			std::cout << "...OK (" << (int) fd[i] << ")" << std::endl;
			if (fd[i] > fd_max)
				fd_max = fd[i];
		}
		tcgetattr(fd[i], &oldtio[i]);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CS8 | CLOCAL | CREAD | CSTOPB;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close(fd[i]);
			fd[i] = -1;
			throw(std::runtime_error("unable to set baudrate !!!"));
			return;
		}
		// activate new settings
		tcflush(fd[i], TCIFLUSH);
		tcsetattr(fd[i], TCSANOW, &newtio);
	}
	std::cout << "[info] fd_max: " << fd_max << std::endl;

	clock_gettime(CLOCK_MONOTONIC, &wake_time);

	reset_counters();
}

void HI_moxa::insert_set_value(int drive_offset, double set_value)
{
	int drive_number;
	drive_number = first_drive_number + drive_offset;
	servo_data[drive_number].buf[0] = 0x00;
	servo_data[drive_number].buf[1] = 0x00;
	servo_data[drive_number].buf[2] = 0x00;
	servo_data[drive_number].buf[3] = 0x00;
	servo_data[drive_number].buf[4] = START_BYTE;
	servo_data[drive_number].buf[5] = COMMAND_MODE_PWM | servo_data[drive_number].command_params;
	struct pwm_St* temp = (pwm_St*) &(servo_data[drive_number].buf[6]);
	//temp->pwm = set_value * (300.0 / 255.0);
	temp->pwm = set_value * (1000.0 / 255.0);

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::insert_set_value(" << drive_offset << ", " << set_value << ")" << std::endl;
#endif
}

int HI_moxa::get_current(int drive_offset)
{
	int ret;

	ret = servo_data[first_drive_number + drive_offset].drive_status.current;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_current(" << drive_offset << ") = " << ret << std::endl;
#endif
	return ret;
}

double HI_moxa::get_increment(int drive_offset)
{
	double ret;
	ret = servo_data[first_drive_number + drive_offset].current_position_inc;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_increment(" << drive_offset << ") = " << ret << std::endl;
#endif
	return ret;
}

long int HI_moxa::get_position(int drive_offset)
{
	int ret;

	ret = servo_data[first_drive_number + drive_offset].current_absolute_position;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_position(" << drive_offset << ") = " << ret << std::endl;
#endif
	return ret;
}

uint64_t HI_moxa::read_write_hardware(void)
{
	static int64_t receive_attempts = 0, receive_timeouts = 0;
	static int error_msg_power_stage = 0;
	bool robot_synchronized;
	bool power_fault;
	bool hardware_read_ok = true;
	bool all_hardware_read = true;
	unsigned int bytes_received[MOXA_SERVOS_NR];
	fd_set rfds;
	uint64_t ret = 0;
	uint8_t drive_number;

	//	std::cout << "[info] pos:";													// ############# Pozycja
	//	 for(drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++){
	//	 std::cout << " " << servo_data[drive_number].current_absolute_position;
	//	 }
	//	 std::cout << std::endl;

	//	std::cout << "[info] przed write: " << std::endl;

	for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
		write(fd[drive_number], servo_data[drive_number].buf, WRITE_BYTES);
		bytes_received[drive_number] = 0;
	}

	//	std::cout << "[info] po write: " << std::endl;


	receive_attempts++;
	//for (int i = 0; i < 7; i++)
	while (1) {
		FD_ZERO(&rfds);
		for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
			if (bytes_received[drive_number] < READ_BYTES) {
				FD_SET(fd[drive_number], &rfds);
			}
		}

		// timeout
		struct timeval timeout;
		timeout.tv_sec = (time_t) 0;
		timeout.tv_usec = 500;
		int select_retval = select(fd_max + 1, &rfds, NULL, NULL, &timeout);
		if (select_retval == 0) {
			receive_timeouts++;
			if (!master.robot_test_mode) {
				std::cout << "[error] communication timeout (" << receive_timeouts << "/" << receive_attempts << "="
						<< (((float) receive_timeouts) / receive_attempts) << ")";

				//<< std::endl;

				//throw(std::runtime_error("communication timeout !!!"));
				/*std::cout << "[error] communication timeout ("
				 << ++receive_timeouts << "/" << receive_attempts << "="
				 << ((float) receive_timeouts / receive_attempts) << ")";
				 //					<< std::endl;*/

				for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
					if (bytes_received[drive_number] < READ_BYTES)
						std::cout << " " << (int) drive_number << "(" << READ_BYTES - bytes_received[drive_number]
								<< ")";
				}
				std::cout << std::endl;
			}

			hardware_read_ok = false;
			break;
		} else {
			all_hardware_read = true;
			for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
				if (FD_ISSET(fd[drive_number], &rfds)) {
					bytes_received[drive_number]
							+= read(fd[drive_number], (char*) (&(servo_data[drive_number].drive_status))
									+ bytes_received[drive_number], READ_BYTES - bytes_received[drive_number]);
				}
				if (bytes_received[drive_number] < READ_BYTES) {
					all_hardware_read = false;
				}
			}
			//		std::cout << all_hardware_read << std::endl;	//############################
			if (all_hardware_read) {
				break;
			}
			//			else{
			//			 for(drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
			//			 if(bytes_received[drive_number] < READ_BYTES){
			//			 std::cout << "Waiting for " << (int)drive_number << std::endl;	//############################
			//			 }
			//			 }
			//			 }
			//			std::cout << std::endl;	//############################
		}
	}

	// Wypelnienie pol odebranymi danymi
	for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {

		// Wypelnienie pol odebranymi danymi
		if (bytes_received[drive_number] >= READ_BYTES) {
			servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
			servo_data[drive_number].current_absolute_position = servo_data[drive_number].drive_status.position;
		}

		// W pierwszym odczycie danych z napedu przyrost pozycji musi byc 0.
		if (servo_data[drive_number].first_hardware_read && hardware_read_ok) {
			servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
			servo_data[drive_number].first_hardware_read = false;
		}

		servo_data[drive_number].current_position_inc = (double) (servo_data[drive_number].current_absolute_position
				- servo_data[drive_number].previous_absolute_position);
	}

	robot_synchronized = false;
	power_fault = false;
	for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
		if (servo_data[drive_number].drive_status.powerStageFault != 0) {
			power_fault = true;
		}
		if (servo_data[drive_number].drive_status.isSynchronized != 0) {
			robot_synchronized = true;
		}
	}

	//zeby zbik3d ruszyl
	if (!master.robot_test_mode) {
		master.controller_state_edp_buf.is_synchronised = robot_synchronized;
	} else {
		master.controller_state_edp_buf.is_synchronised = true;
	}

	master.controller_state_edp_buf.is_robot_blocked = power_fault;
	if (power_fault) {
		if (error_msg_power_stage == 0) {
			master.msg->message(lib::NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
			error_msg_power_stage++;
		}

	} else {
		error_msg_power_stage = 0;
	}

	for (drive_number = first_drive_number; drive_number <= last_drive_number; drive_number++) {
		if (servo_data[drive_number].drive_status.sw1 != 0)
			ret |= (uint64_t) (UPPER_LIMIT_SWITCH << (5 * (drive_number - first_drive_number))); // Zadzialal wylacznik "gorny" krancowy
		if (servo_data[drive_number].drive_status.sw2 != 0)
			ret |= (uint64_t) (LOWER_LIMIT_SWITCH << (5 * (drive_number - first_drive_number))); // Zadzialal wylacznik "dolny" krancowy
		if (servo_data[drive_number].drive_status.swSynchr != 0)
			ret |= (uint64_t) (SYNCHRO_SWITCH_ON << (5 * (drive_number - first_drive_number))); // Zadzialal wylacznik synchronizacji
		if (servo_data[drive_number].drive_status.synchroZero != 0)
			ret |= (uint64_t) (SYNCHRO_ZERO << (5 * (drive_number - first_drive_number))); // Impuls zera rezolwera
		if (servo_data[drive_number].drive_status.overcurrent != 0)
			ret |= (uint64_t) (OVER_CURRENT << (5 * (drive_number - first_drive_number))); // Przekroczenie dopuszczalnego pradu
	}

	while ((wake_time.tv_nsec += COMMCYCLE_TIME_NS) > 1000000000) {
		wake_time.tv_sec += 1;
		wake_time.tv_nsec -= 1000000000;
	}

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);

	return ret;
}

void HI_moxa::reset_counters(void)
{

	for (int i = 0; i < master.number_of_servos; i++) {

		servo_data[i].current_absolute_position = 0L;
		servo_data[i].previous_absolute_position = 0L;
		servo_data[i].current_position_inc = 0.0;

	} // end: for


	//	std::cout << "[func] HI_moxa::reset_counters" << std::endl;
}

void HI_moxa::start_synchro(int drive_offset)
{
	servo_data[first_drive_number + drive_offset].command_params |= COMMAND_PARAM_SYNCHRO;

	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::start_synchro(" << drive_offset << ")" << std::endl;
	//#endif
}

void HI_moxa::finish_synchro(int drive_offset)
{
	servo_data[first_drive_number + drive_offset].command_params &= 0;

	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::finish_synchro(" << drive_offset << ")" << std::endl;
	//#endif
}

bool HI_moxa::is_impulse_zero(int drive_offset)
{
	std::cout << "[func] HI_moxa::is_impulse_zero(" << drive_offset << ")" << std::endl;
	return false;
}

void HI_moxa::reset_position(int drive_offset)
{
	int drive_number;
	drive_number = first_drive_number + drive_offset;
	servo_data[drive_number].current_absolute_position = 0L;
	servo_data[drive_number].previous_absolute_position = 0L;
	servo_data[drive_number].current_position_inc = 0.0;
	servo_data[drive_number].first_hardware_read = true;
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::reset_position(" << drive_offset << ")" << std::endl;
	//#endif
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

