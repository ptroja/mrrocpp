// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota iconveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_RYDZ_H
#define __HI_RYDZ_H

#include <boost/shared_ptr.hpp>
#include "base/edp/HardwareInterface.h"

namespace mrrocpp {
namespace lib {
class periodic_timer;
}
namespace edp {
namespace common {
class motor_driven_effector;
}
namespace hi_rydz {

const uint64_t HARDWARE_ERROR_MASK = 0xE739CCE739CE739CULL;
const uint64_t MASK_RESOLVER_ZERO = 0x3F7BDEF7BDEF7BDEULL;

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

// Polecenia dla sterownikow mikroprocesorowych osi
const int RESET_POSITION_COUNTER = 0x0400; // Zerowanie licznika polozenia
const int RESET_MANUAL_MODE = 0x0800;// Zerowanie pracy recznej
const int RESET_ALARM = 0x0C00; // Zerowanie alarmu sytuacji awaryjnej
const int PROHIBIT_MANUAL_MODE = 0x1000; // Zakaz pracy recznej
const int ALLOW_MANUAL_MODE = 0x1400;// Zezwolenie na prace reczna
const int START_SYNCHRO = 0x1800;// Rozpoczecie synchronizacji
const int FINISH_SYNCHRO = 0x1C00;// Zakoncz synchronizacje osi
const int SET_INT_FREQUENCY = 0x2000;// Ustaw dzielnik czestotliowsci przerwan
const int SET_MAX_CURRENT = 0x2400;// Ustaw prad maksymalny
const int START_CLOCK_INTERRUPTS = 0x5000; // Wlacz przerwania zegarowe
const int STOP_CLOCK_INTERRUPTS = 0x5400; // Wylacz przerwania zegarowe
const int MICROCONTROLLER_MODE = 0x4C00;
const int ZERO_ORDER = 0x0000;
const int STOP_MOTORS = 0x0200;// Zatrzymanie silnikow (W.S. ???)
// ISA_CARD_OFFSET needs to be defined in hi_local.h
const int ADR_OF_SERVO_PTR = 0x305;
const int SERVO_COMMAND1_ADR = 0x200;
const int SERVO_COMMAND2_ADR = 0x202;
const int SERVO_REPLY_STATUS_ADR = 0x200;
const int SERVO_REPLY_INT_ADR = 0x202;
const int SERVO_REPLY_POS_LOW_ADR = 0x204;
const int SERVO_REPLY_POS_HIGH_ADR = 0x206;
const int SERVO_REPLY_REG_1_ADR = 0x208;

class HI_rydz : public common::HardwareInterface
{
public:
	int max_current[lib::MAX_SERVOS_NR];

	HI_rydz(common::motor_driven_effector &_master, int _hi_irq_real, unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high, unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, const int _max_current[]); // Konstruktor

	virtual ~HI_rydz(void); // Destruktor

	virtual bool is_hardware_error(void); // Sprawdzenie czy wystapil blad sprzetowy

	virtual void init();

	virtual void insert_set_value(int drive_number, double set_value);

	virtual int get_current(int drive_number);

	virtual double get_increment(int drive_number);

	virtual long int get_position(int drive_number);

	virtual uint64_t read_write_hardware(void); // Obsluga sprzetu

	virtual void reset_counters(void); // Zerowanie licznikow polozenia

	virtual void start_synchro(int drive_number);

	virtual void finish_synchro(int drive_number);

	virtual bool in_synchro_area(int drive_number);

	virtual bool robot_synchronized();

	virtual bool is_impulse_zero(int drive_number);

	virtual void reset_position(int i);

	virtual int set_parameter(int drive_number, const int parameter, uint32_t new_value);

private:
	int int_id; // Identyfikator obslugi przerwania

	const int hi_irq_real;
	const unsigned short int hi_intr_freq_divider;
	const unsigned int hi_intr_timeout_high;
	const unsigned int hi_first_servo_ptr;
	const unsigned int hi_isa_card_offset;
	const unsigned int hi_intr_generator_servo_ptr;

	boost::shared_ptr<lib::periodic_timer> ptimer;

protected:

	// oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
	int hi_int_wait(common::interrupt_mode_t _interrupt_mode, int lag);

	common::irq_data_t irq_data;

	int measured_current[lib::MAX_SERVOS_NR]; // by Y - zmierzona wartosc pradu
	long int current_absolute_position[lib::MAX_SERVOS_NR]; // aktualne polozenia osi
	long int previous_absolute_position[lib::MAX_SERVOS_NR]; // poprzednie polozenia osi
	double current_position_inc[lib::MAX_SERVOS_NR]; // aktualny przyrost polozenia

	bool trace_resolver_zero;

	common::control_a_dof robot_control[lib::MAX_SERVOS_NR];
	common::status_of_a_dof robot_status[lib::MAX_SERVOS_NR];
};

#ifdef __cplusplus
extern "C" {
#endif
const struct sigevent * int_handler(void *arg, int id);
#ifdef __cplusplus
}
#endif

} // namespace hi_rydz
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
