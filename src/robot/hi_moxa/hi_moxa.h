/* TODO:
 *
 * inicjalizacja struktur servo_data w konstruktorze hi_moxa
 * przekazanie do konstruktora hi_moxa danych o ilosci i numerach portow
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

#define USLEEP_US 500000
#define STATUS_DISP_T 100

#include "config.h"

#ifdef HAVE_POSIX_TIMERS
#include <ctime>
#else
#include <boost/thread/thread_time.hpp>
#endif

#include <boost/date_time/posix_time/posix_time_types.hpp>

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

const long COMMCYCLE_TIME_NS = 2000000;
const boost::posix_time::time_duration COMMCYCLE_TIME = boost::posix_time::millisec(2);

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class HI_moxa : public common::HardwareInterface
{

public:
	HI_moxa(common::motor_driven_effector &_master, int last_drive_n, std::vector<std::string> ports); // Konstruktor

	~HI_moxa();

	virtual void init();
	virtual void insert_set_value(int drive_number, double set_value);
	virtual int get_current(int drive_number);
	virtual double get_increment(int drive_number);
	virtual long int get_position(int drive_number);
	virtual uint64_t read_write_hardware(void); // Obsluga sprzetu
	virtual int  set_parameter(int drive_number, const int parameter, uint32_t new_value);
	virtual void reset_counters(void); // Zerowanie licznikow polozenia
	virtual void start_synchro(int drive_number);
	virtual void finish_synchro(int drive_number);
	virtual bool in_synchro_area(int drive_number);
	virtual bool robot_synchronized();
	virtual void set_command_param(int drive_offset, uint8_t param);

	virtual bool is_impulse_zero(int drive_offset);
	virtual void reset_position(int drive_offset);

private:
	#if defined(B921600)
	static const speed_t BAUD = B921600;
	#else
	static const speed_t BAUD = 921600;
	#endif

	void write_read(int fd, char* buf, unsigned int w_len, unsigned int r_len);

	const std::size_t last_drive_number;
	std::vector<std::string> port_names;
	int fd[MOXA_SERVOS_NR], fd_max;
	struct servo_St servo_data[MOXA_SERVOS_NR];
	struct termios oldtio[MOXA_SERVOS_NR];
#ifdef HAVE_POSIX_TIMERS
	struct timespec wake_time;
#else
	boost::posix_time::ptime wake_time;
#endif

}; // koniec: class hardware_interface

} // namespace hi_moxa
} // namespace edp
} // namespace mrrocpp

#endif // __HI_MOXA_H
