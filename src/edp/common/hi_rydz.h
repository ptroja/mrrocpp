// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota iconveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_RYDZ_H
#define __HI_RYDZ_H

#include <stdint.h>
#include "edp/common/edp_e_motor_driven.h"

namespace mrrocpp {
namespace edp {
namespace common {

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

// tryby obslugi przerwania
typedef enum INTERRUPT_MODE {
	INT_EMPTY, // obluga pusta
	INT_SERVOING, // tryb regulacji osi
	INT_SINGLE_COMMAND, // do synchronizacji, inicjacji, etc.
	INT_CHECK_STATE // do odczytu stanu z adresu 0x220
} interrupt_mode_t;

// ISA_CARD_OFFSET needs to be defined in hi_local.h
#define ADR_OF_SERVO_PTR          (0x305)
#define SERVO_COMMAND1_ADR        (0x200)
#define SERVO_COMMAND2_ADR        (0x202)
#define SERVO_REPLY_STATUS_ADR    (0x200)
#define SERVO_REPLY_INT_ADR       (0x202)
#define SERVO_REPLY_POS_LOW_ADR   (0x204)
#define SERVO_REPLY_POS_HIGH_ADR  (0x206)
#define SERVO_REPLY_REG_1_ADR     (0x208)

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
    uint16_t register_adress;  // adres rejestru dla trybu INT_SINGLE_COMMAND
    uint16_t value; // wartosc do wstawienia dla trybu INT_SINGLE_COMMAND

    long int current_absolute_position[MAX_SERVOS_NR];
    control_a_dof robot_control[MAX_SERVOS_NR];
    status_of_a_dof robot_status[MAX_SERVOS_NR];

    uint64_t hardware_error;
};

typedef struct _irq_data {
#ifdef __QNXNTO__
	struct sigevent event; // sygnalilzacja przerwania dla glownego watku
#endif
	common::motor_data md; // Dane przesylane z/do funkcji obslugi przerwania
} irq_data_t;


class hardware_interface
{
public:
	motor_driven_effector &master;

	int max_current[MAX_SERVOS_NR];
    hardware_interface (motor_driven_effector &_master,
    		int _hi_irq_real,
    		unsigned short int _hi_intr_freq_divider,
    		unsigned int _hi_intr_timeout_high,
    		unsigned int _hi_first_servo_ptr,
    		unsigned int _hi_intr_generator_servo_ptr,
    		unsigned int _hi_isa_card_offset,
    		const int _max_current[]);    // Konstruktor

    virtual ~hardware_interface(void );   // Destruktor
    virtual bool is_hardware_error(void); // Sprawdzenie czy wystapil blad sprzetowy

    void init();

    void insert_set_value ( int drive_number, double set_value);

    int get_current ( int drive_number ) const;

    double get_increment ( int drive_number ) const;

    long int get_position ( int drive_number ) const;

    virtual uint64_t read_write_hardware ( void );    // Obsluga sprzetu
    virtual void reset_counters ( void );  // Zerowanie licznikow polozenia

    virtual void start_synchro ( int drive_number ) ;

    virtual void finish_synchro ( int drive_number ) ;

    // oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
    int hi_int_wait (interrupt_mode_t _interrupt_mode, int lag);

    bool is_impulse_zero ( int drive_number ) const;

    void reset_position (int i);

private:
    int int_id;		// Identyfikator obslugi przerwania

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
    irq_data_t irq_data;

    int meassured_current[MAX_SERVOS_NR]; // by Y - zmierzona wartosc pradu
    long int current_absolute_position[MAX_SERVOS_NR];  // aktualne polozenia osi
    long int previous_absolute_position[MAX_SERVOS_NR]; // poprzednie polozenia osi
    double current_position_inc[MAX_SERVOS_NR];         // aktualny przyrost polozenia

    bool trace_resolver_zero;

    control_a_dof robot_control[MAX_SERVOS_NR];
    status_of_a_dof robot_status[MAX_SERVOS_NR];
};

#ifdef __cplusplus
extern "C"
{
#endif
    const struct sigevent * int_handler (void *arg, int id);
#ifdef __cplusplus
}
#endif

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
