/*
 * hi_moxa.h
 *
 *  Created on: Nov 14, 2011
 *      Author: mwalecki
 */

#ifndef __HI_MOXA_H
#define __HI_MOXA_H

#include <termios.h>
#include <string>
#include <vector>
#include <stdint.h>

// // // // // // // //
// TERMINAL INFO
//#define T_INFO_FUNC
//#define T_INFO_CALC

#define STATUS_DISP_T 100

#include "base/lib/periodic_timer.h"
#include "base/edp/HardwareInterface.h"
#include "robot/hi_moxa/hi_moxa_combuf.h"

namespace mrrocpp {
namespace edp {
namespace common {
class motor_driven_effector;
}
namespace hi_moxa {

const std::size_t WRITE_BYTES = 10;
const std::size_t READ_BYTES = 8;
const std::size_t MOXA_SERVOS_NR = 8;
const int MAX_PARAM_SET_ATTEMPTS = 3;
const int MAX_COMM_TIMEOUTS = 3;
const int FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT = 4;

const int VOLTAGE = 48.0;

const unsigned long COMMCYCLE_TIME_NS = 2000000;

/*!
 * @brief hardware interface class
 *
 * @author mwalecki
 * @ingroup edp
 */
class HI_moxa : public common::HardwareInterface
{

public:
	/**
	 * @brief constructor
	 * @param &_master			master effector
	 * @param last_drive_n		number of drives
	 * @param ports				vector of serial port names
	 * @param *max_increments	tab of max allowed motor increments
	 */
	HI_moxa(common::motor_driven_effector &_master, int last_drive_n, std::vector <std::string> ports, const double* max_increments); // Konstruktor
	/**
	 * @brief destructor
	 */
	~HI_moxa();
	/**
	 * @brief initialization of hardware interface
	 * opens serial ports, writes init values to data structures
	 */
	virtual void init();
	/**
	 * @brief write pwm to communication buffer
	 * @param drive_number		number of drive
	 * @param set_value			pwm value
	 * writes desired pwm value to drive's communication buffer
	 */
	virtual void insert_set_value(int drive_number, double set_value);
	/**
	 * @brief read motor current from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual int get_current(int drive_number);

	/**
	 * @brief read voltage aplitude
	 * @param drive_number		number of drive
	 */
	virtual float get_voltage(int drive_number);

	/**
	 * @brief read motor increment from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual double get_increment(int drive_number);
	/**
	 * @brief read motor position from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual long int get_position(int drive_number);
	/**
	 * @brief do communication cycle
	 * sends data in communication buffer to motor controllers,
	 * waits 500us for answers from controllers,
	 * writes received data to communication buffer.
	 */
	virtual uint64_t read_write_hardware(void);
	/**
	 * @brief send parameter to motor driver
	 * @param drive_number		number of drive
	 * @param parameter			parameter type
	 * @param new_value			parameter value
	 */
	virtual int set_parameter(int drive_number, const int parameter, uint32_t new_value);
	/**
	 * @brief reset all motor positions and position increments in communication buffer
	 */
	virtual void reset_counters(void);
	/**
	 * @brief start synchronization procedure
	 * @param drive_number		number of drive
	 * command motor controller to look for 'synchro zero' signal
	 */
	virtual void start_synchro(int drive_number);
	/**
	 * @brief finish synchronization procedure
	 * @param drive_number		number of drive
	 */
	virtual void finish_synchro(int drive_number);
	/**
	 * @brief read 'in synchro area' flag from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual bool in_synchro_area(int drive_number);
	/**
	 * @brief read 'all robots synchronized' flag from communication buffer
	 */
	virtual bool robot_synchronized();
	/**
	 * @brief set parameter to send with next command
	 * @param drive_number		number of drive
	 * @param param			parameter type
	 */
	virtual void set_command_param(int drive_offset, uint8_t param);
	/**
	 * @brief read 'is impulse zero' flag from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual bool is_impulse_zero(int drive_offset);
	/**
	 * @brief reset motor position in communication buffer
	 * @param drive_number		number of drive
	 */
	virtual void reset_position(int drive_offset);

private:
	/// communication baud rate (bps)
#if defined(B921600)
	static const speed_t BAUD = B921600;
#else
	static const speed_t BAUD = 921600;
#endif
	/// (number of drives)-1
	const std::size_t last_drive_number;
	/// vector of serial port names
	std::vector <std::string> port_names;
	/// tab of max allowed motor position increments
	const double* ridiculous_increment;
	/// tab of port designators
	int fd[MOXA_SERVOS_NR], fd_max;
	/// tab of data buffers
	struct servo_St servo_data[MOXA_SERVOS_NR];
	struct termios oldtio[MOXA_SERVOS_NR];

	/// periodic timer used for generating read_write_hardware time base
	lib::periodic_timer ptimer;
};
// endof: class hardware_interface

}// namespace hi_moxa
} // namespace edp
} // namespace mrrocpp

#endif // __HI_MOXA_H
