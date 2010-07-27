/*
 * HardwareInterface.h
 *
 *  Created on: 19-07-2010
 *      Author: konradb3
 */

#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_

#include <inttypes.h>

namespace mrrocpp {
namespace edp {
namespace common {

class HardwareInterface {
public:
	HardwareInterface()
	{

	}
	virtual ~HardwareInterface()
	{

	}

	virtual void init() = 0;
	virtual void insert_set_value(int drive_number, double set_value) = 0;
	virtual int get_current(int drive_number) = 0;
	virtual double get_increment(int drive_number) = 0;
	virtual long int get_position(int drive_number) = 0;
	virtual uint64_t read_write_hardware(void) = 0; // Obsluga sprzetu
	virtual void reset_counters(void) = 0; // Zerowanie licznikow polozenia
	virtual void start_synchro(int drive_number) = 0;
	virtual void finish_synchro(int drive_number) = 0;

	virtual bool is_impulse_zero(int drive_number) = 0;
	virtual void reset_position(int i) = 0;

};

}
}
}

#endif /* HARDWAREINTERFACE_H_ */
