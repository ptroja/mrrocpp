// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

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

#define PORT "/dev/ser"
#define BAUD 921600
#define START_BYTE '#'
#define WRITE_BYTES 10
#define READ_BYTES 8

#define COMMCYCLE_TIME_NS	2000000

namespace mrrocpp {
namespace edp {
namespace common {
class motor_driven_effector;
}
namespace hi_moxa {

class motor_driven_effector;

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class HI_moxa : public common::HardwareInterface
{

public:

	HI_moxa(common::motor_driven_effector &_master); // Konstruktor
	~HI_moxa();

	virtual void init();
	virtual void insert_set_value(int drive_number, double set_value);
	virtual int get_current(int drive_number);
	virtual double get_increment(int drive_number);
	virtual long int get_position(int drive_number);
	virtual uint64_t read_write_hardware(void); // Obsluga sprzetu
	virtual void reset_counters(void); // Zerowanie licznikow polozenia
	virtual void start_synchro(int drive_number);
	virtual void finish_synchro(int drive_number);

	virtual bool is_impulse_zero(int drive_number);
	virtual void reset_position(int i);

protected:
private:

	void write_read(int fd, char* buf, unsigned int w_len, unsigned int r_len);

	int fd[8];
	struct termios oldtio[8];
	struct timespec wake_time;

	char buf[30];
	uint8_t command_params[8];
	struct status_St drive_status[8];
	int32_t position_offset[8];
	int32_t current_absolute_position[8];
	int32_t previous_absolute_position[8];
	double current_position_inc[8];
	bool first_hardware_read[8];
	bool trace_resolver_zero[8];

}; // koniec: class hardware_interface

} // namespace hi_moxa
} // namespace edp
} // namespace mrrocpp

#endif // __HI_MOXA_H
