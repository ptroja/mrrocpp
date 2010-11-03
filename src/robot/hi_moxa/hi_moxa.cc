#include "robot/hi_moxa/hi_moxa.h"

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

HI_moxa::HI_moxa(common::motor_driven_effector &_master, int last_drive_n, std::vector<std::string> ports) :
	common::HardwareInterface(_master), last_drive_number(last_drive_n), port_names(ports)
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
	for (unsigned int i = 0; i <= last_drive_number; i++) {
		if (fd[i] > 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close(fd[i]);
		}
	}
}

void HI_moxa::init()
{

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::init()" << std::endl;
#endif
	// inicjalizacja zmiennych
	for (unsigned int i = 0; i <= last_drive_number; i++) {
		servo_data[i].first_hardware_read = true;
		servo_data[i].command_params = 0;
		for(int j=0; j<SERVO_ST_BUF_LEN; j++)
			servo_data[i].buf[j] = 0;
	}

	// informacja o stanie robota
	master.controller_state_edp_buf.is_power_on = true;
	master.controller_state_edp_buf.is_robot_blocked = false;

	if (master.robot_test_mode) {
		// domyslnie robot jest zsynchronizowany
		master.controller_state_edp_buf.is_synchronised = true;
		// informacja o stanie robota
		master.controller_state_edp_buf.is_power_on = true;
		master.controller_state_edp_buf.is_robot_blocked = false;

		clock_gettime(CLOCK_MONOTONIC, &wake_time);
		reset_counters();
		return;
	} // end test mode

	// domyslnie robot nie jest zsynchronizowany
	master.controller_state_edp_buf.is_synchronised = false;



	fd_max = 0;
	for (unsigned int i = 0; i <= last_drive_number; i++) {
		std::cout << "[info] opening port : " << port_names[i].c_str();
		fd[i] = open(port_names[i].c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd[i] < 0) {
			std::cout << std::endl << "[error] Nie wykryto sprzetu! fd == " << (int) fd[i] << std::endl;
			//throw(std::runtime_error("unable to open device!!!"));
		} else {
			std::cout << "...OK" << std::endl;
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

	clock_gettime(CLOCK_MONOTONIC, &wake_time);

	reset_counters();
}

void HI_moxa::insert_set_value(int drive_number, double set_value)
{
	servo_data[drive_number].buf[0] = 0x00;
	servo_data[drive_number].buf[1] = 0x00;
	servo_data[drive_number].buf[2] = 0x00;
	servo_data[drive_number].buf[3] = 0x00;
	servo_data[drive_number].buf[4] = START_BYTE;
	servo_data[drive_number].buf[5] = COMMAND_MODE_PWM | servo_data[drive_number].command_params;
	struct pwm_St* temp = (pwm_St*) &(servo_data[drive_number].buf[6]);
	// Nowa karta sterownika: -1000..+1000, stara karta: -255..+255
	temp->pwm = set_value * (1000.0 / 255.0);

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::insert_set_value(" << drive_number << ", " << set_value << ")" << std::endl;
#endif
}

int HI_moxa::get_current(int drive_number)
{
	int ret;

	ret = servo_data[drive_number].drive_status.current;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_current(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

double HI_moxa::get_increment(int drive_number)
{
	double ret;
	ret = servo_data[drive_number].current_position_inc;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_increment(" << drive_number << ") = " << ret << std::endl;
#endif
	return ret;
}

long int HI_moxa::get_position(int drive_number)
{
	int ret;

	ret = servo_data[drive_number].current_absolute_position;

#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::get_position(" << drive_number << ") = " << ret << std::endl;
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
	static int status_disp_cnt = 0;

	// test mode
	if (master.robot_test_mode) {
		while ((wake_time.tv_nsec += COMMCYCLE_TIME_NS) > 1000000000) {
			wake_time.tv_sec += 1;
			wake_time.tv_nsec -= 1000000000;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
		return ret;
	}// end test mode

	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
		write(fd[drive_number], servo_data[drive_number].buf, WRITE_BYTES);
		bytes_received[drive_number] = 0;
	}

	receive_attempts++;

	while (1) {
		FD_ZERO(&rfds);
		for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
			if (bytes_received[drive_number] < READ_BYTES) {
				FD_SET(fd[drive_number], &rfds);
			}
		}

		struct timeval timeout;
		timeout.tv_sec = (time_t) 0;
		timeout.tv_usec = 500;
		int select_retval = select(fd_max + 1, &rfds, NULL, NULL, &timeout);
		if (select_retval == 0) {
			receive_timeouts++;
			std::cout << "[error] communication timeout (" << receive_timeouts << "/" << receive_attempts << "="
					<< (((float) receive_timeouts) / receive_attempts) << ")";

			for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
				if (bytes_received[drive_number] < READ_BYTES)
					std::cout << " " << (int) drive_number << "(" << READ_BYTES - bytes_received[drive_number] << ")";
			}
			std::cout << std::endl;
			hardware_read_ok = false;
			break;
		} else {
			all_hardware_read = true;
			for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
				if (FD_ISSET(fd[drive_number], &rfds)) {
					bytes_received[drive_number]
							+= read(fd[drive_number], (char*) (&(servo_data[drive_number].drive_status))
									+ bytes_received[drive_number], READ_BYTES - bytes_received[drive_number]);
				}
				if (bytes_received[drive_number] < READ_BYTES) {
					all_hardware_read = false;
				}
			}
			if (all_hardware_read) {
				break;
			}
		}
	}

	// Wypelnienie pol odebranymi danymi
	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {

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

	robot_synchronized = true;
	power_fault = false;
	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
		if (servo_data[drive_number].drive_status.powerStageFault != 0) {
			power_fault = true;
		}
		if (servo_data[drive_number].drive_status.isSynchronized == 0) {
			robot_synchronized = false;
		}
	}

	master.controller_state_edp_buf.is_synchronised = robot_synchronized;
	master.controller_state_edp_buf.is_robot_blocked = power_fault;
	if (power_fault) {
		if (error_msg_power_stage == 0) {
			master.msg->message(lib::NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
			error_msg_power_stage++;
		}

	} else {
		error_msg_power_stage = 0;
	}

	for (drive_number = 0; drive_number <= last_drive_number; drive_number++) {
		if (servo_data[drive_number].drive_status.sw1 != 0)
			ret |= (uint64_t) (UPPER_LIMIT_SWITCH << (5 * (drive_number))); // Zadzialal wylacznik "gorny" krancowy
		if (servo_data[drive_number].drive_status.sw2 != 0)
			ret |= (uint64_t) (LOWER_LIMIT_SWITCH << (5 * (drive_number ))); // Zadzialal wylacznik "dolny" krancowy
		if (servo_data[drive_number].drive_status.swSynchr != 0)
			ret |= (uint64_t) (SYNCHRO_SWITCH_ON << (5 * (drive_number))); // Zadzialal wylacznik synchronizacji
		if (servo_data[drive_number].drive_status.synchroZero != 0)
			ret |= (uint64_t) (SYNCHRO_ZERO << (5 * (drive_number))); // Impuls zera rezolwera
		if (servo_data[drive_number].drive_status.overcurrent != 0)
			ret |= (uint64_t) (OVER_CURRENT << (5 * (drive_number))); // Przekroczenie dopuszczalnego pradu
	}

	if(status_disp_cnt++ == STATUS_DISP_T)
	{
	//	std::cout << "[info] current[0] = " << (int) servo_data[0].drive_status.current << std::endl;
		status_disp_cnt = 0;
	}

	while ((wake_time.tv_nsec += COMMCYCLE_TIME_NS) > 1000000000) {
		wake_time.tv_sec += 1;
		wake_time.tv_nsec -= 1000000000;
	}

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);

	return ret;
}

int HI_moxa::set_parameter(int drive_number, const int parameter, uint32_t new_value)
{
	char tx_buf[SERVO_ST_BUF_LEN];
	char rx_buf[SERVO_ST_BUF_LEN];
	fd_set rfds;
	int bytes_received=0;


	tx_buf[0] = 0x00;
	tx_buf[1] = 0x00;
	tx_buf[2] = 0x00;
	tx_buf[3] = 0x00;
	tx_buf[4] = START_BYTE;
	tx_buf[5] = COMMAND_SET_PARAM | parameter;
	union param_Un* temp = (param_Un*) &(tx_buf[6]);
	temp->largest = 0;

	switch(parameter)
	{
	case PARAM_SYNCHRONIZED:
		temp->synchronized = new_value;
	break;
	case PARAM_MAXCURRENT:
		temp->maxcurrent = new_value;
	break;
	case PARAM_PID_POS_P:
	case PARAM_PID_POS_I:
	case PARAM_PID_POS_D:
	case PARAM_PID_CURR_P:
	case PARAM_PID_CURR_I:
	case PARAM_PID_CURR_D:
		temp->pid_coeff = new_value;
	break;
	case PARAM_DRIVER_MODE:
		temp->driver_mode = new_value;
	break;
	default:
		std::cout << "[error] HI_moxa::set_parameter() invalid parameter" << std::endl;
		return -1;
	break;
	}


	for(int param_set_attempt = 0; param_set_attempt < MAX_PARAM_SET_ATTEMPTS; param_set_attempt++)
	{
		write(fd[drive_number], tx_buf, WRITE_BYTES);
		bytes_received = 0;

		FD_ZERO(&rfds);
		FD_SET(fd[drive_number], &rfds);

		for(int i=0; i<3; i++)
		{
			struct timeval timeout;
			timeout.tv_sec = (time_t) 0;
			timeout.tv_usec = 500;
			int select_retval = select(fd[drive_number] + 1, &rfds, NULL, NULL, &timeout);
			if (select_retval == 0) {
				std::cout << "[error] param set ack timeout for drive (" << drive_number << ")" << std::endl;
			} else {
				bytes_received += read(fd[drive_number], rx_buf + bytes_received, READ_BYTES - bytes_received);

				if(bytes_received == READ_BYTES)
					return 0;
			}
		}
		usleep(2000);
	}
	return 1;
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

void HI_moxa::start_synchro(int drive_number)
{
	servo_data[drive_number].command_params |= COMMAND_PARAM_SYNCHRO;

	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::start_synchro(" << drive_number << ")" << std::endl;
	//#endif
}

void HI_moxa::finish_synchro(int drive_number)
{
	servo_data[drive_number].command_params &= ~COMMAND_PARAM_SYNCHRO;

	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::finish_synchro(" << drive_number << ")" << std::endl;
	//#endif
}

bool HI_moxa::in_synchro_area(int drive_number)
{
	return (servo_data[drive_number].drive_status.swSynchr != 0);
}

bool HI_moxa::robot_synchronized()
{
	bool ret = true;
	for (int i = 0; i <= last_drive_number; i++) {
		if (servo_data[i].drive_status.isSynchronized == 0) {
			ret = false;
		}
	}
	return ret;
}

void HI_moxa::set_command_param(int drive_number, uint8_t param)
{
	servo_data[drive_number].command_params |= param;
}

bool HI_moxa::is_impulse_zero(int drive_number)
{
	std::cout << "[func] HI_moxa::is_impulse_zero(" << drive_number << ")" << std::endl;
	return false;
}

void HI_moxa::reset_position(int drive_number)
{
	servo_data[drive_number].current_absolute_position = 0L;
	servo_data[drive_number].previous_absolute_position = 0L;
	servo_data[drive_number].current_position_inc = 0.0;
	servo_data[drive_number].first_hardware_read = true;
	//#ifdef T_INFO_FUNC
	std::cout << "[func] HI_moxa::reset_position(" << drive_number << ")" << std::endl;
	//#endif
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

