// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_SARKOFAG_H
#define __HI_LOCAL_SARKOFAG_H

// // // // // // // //
// TERMINAL INFO
//#define T_INFO_FUNC
//#define T_INFO_CALC

#define USLEEP_US 500000

#include "base/edp/HardwareInterface.h"
#include "edp_sarkofag_combuf.h"

#include <inttypes.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <string>



#define PORT "/dev/ser"
#define BAUD 921600
#define START_BYTE '#'
#define WRITE_BYTES 10
#define READ_BYTES 8

#define COMMCYCLE_TIME_NS	2000000

namespace mrrocpp {
namespace edp {
namespace sarkofag {

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

class HI_moxa: public common::HardwareInterface {

public:
	HI_moxa(); // Konstruktor
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

	struct status_St sarkofag_status;
	char buf[30];

}; // koniec: class hardware_interface

} // namespace sarkofag
} // namespace edp
} // namespace mrrocpp

#endif // __HI_LOCAL_SARKOFAG_H
