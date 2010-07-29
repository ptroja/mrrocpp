// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota iconveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_RYDZ_H
#define __HI_RYDZ_H

#include <stdint.h>
#include "base/edp/edp_e_motor_driven.h"
#include "base/edp/HardwareInterface.h"

namespace mrrocpp {
namespace edp {
namespace common {

const uint64_t HARDWARE_ERROR_MASK = 0xE739CCE739CE739CULL;
const uint64_t MASK_RESOLVER_ZERO = 0x3F7BDEF7BDEF7BDEULL;

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

// Polecenia dla sterownikow mikroprocesorowych osi
#define RESET_POSITION_COUNTER           0x0400 // Zerowanie licznika polozenia
#define RESET_MANUAL_MODE                0x0800 // Zerowanie pracy recznej
#define RESET_ALARM                      0x0C00 // Zerowanie alarmu sytuacji awaryjnej
#define PROHIBIT_MANUAL_MODE             0x1000 // Zakaz pracy recznej
#define ALLOW_MANUAL_MODE                0x1400 // Zezwolenie na prace reczna
#define START_SYNCHRO                    0x1800 // Rozpoczecie synchronizacji
#define FINISH_SYNCHRO                   0x1C00 // Zakoncz synchronizacje osi
#define SET_INT_FREQUENCY                0x2000 // Ustaw dzielnik czestotliowsci przerwan
#define SET_MAX_CURRENT                  0x2400 // Ustaw prad maksymalny
#define START_CLOCK_INTERRUPTS           0x5000 // Wlacz przerwania zegarowe
#define STOP_CLOCK_INTERRUPTS            0x5400 // Wylacz przerwania zegarowe
#define MICROCONTROLLER_MODE             0x4C00
#define ZERO_ORDER                       0x0000
#define STOP_MOTORS                      0x0200 // Zatrzymanie silnikow (W.S. ???)

// ISA_CARD_OFFSET needs to be defined in hi_local.h
#define ADR_OF_SERVO_PTR          (0x305)
#define SERVO_COMMAND1_ADR        (0x200)
#define SERVO_COMMAND2_ADR        (0x202)
#define SERVO_REPLY_STATUS_ADR    (0x200)
#define SERVO_REPLY_INT_ADR       (0x202)
#define SERVO_REPLY_POS_LOW_ADR   (0x204)
#define SERVO_REPLY_POS_HIGH_ADR  (0x206)
#define SERVO_REPLY_REG_1_ADR     (0x208)

class HI_rydz : public HardwareInterface
{
public:

	int max_current[MAX_SERVOS_NR];
			HI_rydz(motor_driven_effector &_master, int _hi_irq_real, unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high, unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, const int _max_current[]); // Konstruktor

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

	virtual bool is_impulse_zero(int drive_number);

	virtual void reset_position(int i);

private:
	int int_id; // Identyfikator obslugi przerwania

	const int hi_irq_real;
	const unsigned short int hi_intr_freq_divider;
	const unsigned int hi_intr_timeout_high;
	const unsigned int hi_first_servo_ptr;
	const unsigned int hi_isa_card_offset;
	const unsigned int hi_intr_generator_servo_ptr;

	//! periodic timer
	timer_t timerid;

	//! periodic timer signal mask
	sigset_t mask;

protected:

	// oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
	int hi_int_wait(interrupt_mode_t _interrupt_mode, int lag);

	irq_data_t irq_data;

	int meassured_current[MAX_SERVOS_NR]; // by Y - zmierzona wartosc pradu
	long int current_absolute_position[MAX_SERVOS_NR]; // aktualne polozenia osi
	long int previous_absolute_position[MAX_SERVOS_NR]; // poprzednie polozenia osi
	double current_position_inc[MAX_SERVOS_NR]; // aktualny przyrost polozenia

	bool trace_resolver_zero;

	control_a_dof robot_control[MAX_SERVOS_NR];
	status_of_a_dof robot_status[MAX_SERVOS_NR];
};

#ifdef __cplusplus
extern "C" {
#endif
const struct sigevent * int_handler(void *arg, int id);
#ifdef __cplusplus
}
#endif

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
