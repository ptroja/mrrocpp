/* TODO:
 *
 * inicjalizacja struktur servo_data w konstruktorze hi_moxa
 * przekazanie do konstruktora hi_moxa danych o ilosci i numerach portow
 */

#ifndef __HI_MOXA_H
#define __HI_MOXA_H

// // // // // // // //
// TERMINAL INFO
//#define T_INFO_FUNC
//#define T_INFO_CALC

#define USLEEP_US 500000

#include "base/edp/HardwareInterface.h"
#include "robot/hi_moxa/hi_moxa_combuf.h"
//#include "base/edp/edp_e_motor_driven.h"

//#include "edp_e_sarkofag.h"

#include <stdint.h>
#include <termios.h>
#include <ctime>

namespace mrrocpp {
namespace edp {
namespace common {
class motor_driven_effector;
}
namespace hi_moxa {

const std::string PORT = "/dev/ser";
const int BAUD = 921600;
const int WRITE_BYTES = 10;
const int READ_BYTES = 8;
const char INIT_PORT_CHAR = 50;

const int MOXA_SERVOS_NR = 8;

const long COMMCYCLE_TIME_NS = 2000000;

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class HI_moxa : public common::HardwareInterface
{

public:

	HI_moxa(common::motor_driven_effector &_master, int first_drive_n, int last_drive_n); // Konstruktor
	~HI_moxa();

	virtual void init();
	virtual void insert_set_value(int drive_offset, double set_value);
	virtual int get_current(int drive_offset);
	virtual double get_increment(int drive_offset);
	virtual long int get_position(int drive_offset);
	virtual uint64_t read_write_hardware(void); // Obsluga sprzetu
	virtual void reset_counters(void); // Zerowanie licznikow polozenia
	virtual void start_synchro(int drive_offset);
	virtual void finish_synchro(int drive_offset);
	virtual void set_command_param(int drive_offset, uint8_t param);

	virtual bool is_impulse_zero(int drive_offset);
	virtual void reset_position(int drive_offset);

protected:
private:

	void write_read(int fd, char* buf, unsigned int w_len, unsigned int r_len);

	int fd[MOXA_SERVOS_NR], fd_max;
	int first_drive_number, last_drive_number;
	struct servo_St servo_data[MOXA_SERVOS_NR];
	struct termios oldtio[MOXA_SERVOS_NR];
	struct timespec wake_time;

}; // koniec: class hardware_interface

} // namespace hi_moxa
} // namespace edp
} // namespace mrrocpp

#endif // __HI_MOXA_H
