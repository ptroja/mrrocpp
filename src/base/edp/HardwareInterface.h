/*
 * HardwareInterface.h
 *
 *  Created on: 19-07-2010
 *      Author: konradb3
 */

#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_

#include <stdint.h>

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace edp {
namespace common {

class motor_driven_effector;

// tryby obslugi przerwania
typedef enum INTERRUPT_MODE
{
	INT_EMPTY, // obluga pusta
	INT_SERVOING, // tryb regulacji osi
	INT_SINGLE_COMMAND, // do synchronizacji, inicjacji, etc.
	INT_CHECK_STATE
// do odczytu stanu z adresu 0x220
} interrupt_mode_t;

struct control_a_dof
{
	uint16_t adr_offset_plus_0;
	uint16_t adr_offset_plus_2;
};

struct status_of_a_dof
{
	uint16_t adr_offset_plus_0;
	uint16_t adr_offset_plus_2;
	uint16_t adr_offset_plus_4;
	uint16_t adr_offset_plus_6;
};

struct motor_data
{
	bool is_synchronised; // czy robot jest zsynchronizowany
	bool is_power_on; // czy wzmacniacze mocy sa wlaczone
	bool is_robot_blocked; // czy robot jest zablokowany

	interrupt_mode_t interrupt_mode;
	uint8_t card_adress; // adres karty dla trybu INT_SINGLE_COMMAND
	uint16_t register_adress; // adres rejestru dla trybu INT_SINGLE_COMMAND
	uint16_t value; // wartosc do wstawienia dla trybu INT_SINGLE_COMMAND

	long int current_absolute_position[lib::MAX_SERVOS_NR];
	control_a_dof robot_control[lib::MAX_SERVOS_NR];
	status_of_a_dof robot_status[lib::MAX_SERVOS_NR];

	uint64_t hardware_error;
};

typedef struct _irq_data
{

	common::motor_data md; // Dane przesylane z/do funkcji obslugi przerwania
} irq_data_t;

class HardwareInterface
{
public:
	HardwareInterface(motor_driven_effector &_master) :
		master(_master)
	{

	}
	virtual ~HardwareInterface()
	{

	}

	motor_driven_effector &master;
	virtual void init() = 0;
	virtual void insert_set_value(int drive_number, double set_value) = 0;
	virtual int get_current(int drive_number) = 0;
	virtual double get_increment(int drive_number) = 0;
	virtual long int get_position(int drive_number) = 0;
	virtual uint64_t read_write_hardware(void) = 0; // Obsluga sprzetu
	virtual void reset_counters(void) = 0; // Zerowanie licznikow polozenia
	virtual void start_synchro(int drive_number) = 0;
	virtual void finish_synchro(int drive_number) = 0;
	virtual bool in_synchro_area(int drive_number) = 0;
	virtual bool robot_synchronized() = 0;

	virtual bool is_impulse_zero(int drive_number) = 0;
	virtual void reset_position(int i) = 0;

	virtual int set_parameter(int drive_number, const int parameter, uint32_t new_value) = 0;
};

}
}
}

#endif /* HARDWAREINTERFACE_H_ */
