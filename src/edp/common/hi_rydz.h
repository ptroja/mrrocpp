// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota iconveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_RYDZ_H
#define __HI_RYDZ_H

#include <stdint.h>
#include "edp/common/edp_e_manip_and_conv.h"

namespace mrrocpp {
namespace edp {
namespace common {

struct control_a_dof
{
    lib::WORD adr_offset_plus_0;
    lib::WORD adr_offset_plus_2;
};

struct status_of_a_dof
{
    lib::WORD adr_offset_plus_0;
    lib::WORD adr_offset_plus_2;
    lib::WORD adr_offset_plus_4;
    lib::WORD adr_offset_plus_6;
    lib::WORD adr_offset_plus_8;
    lib::WORD adr_offset_plus_a;
};


struct motor_data
{
    bool is_synchronised; // czy robot jest zsynchronizowany
    bool is_power_on; // czy wzmacniacze mocy sa wlaczone
    bool is_robot_blocked; // czy robot jest zablokowany

    int interrupt_mode;
    lib::BYTE card_adress; // adres karty dla trybu INT_SINGLE_COMMAND
    lib::WORD register_adress;  // adres rejestru dla trybu INT_SINGLE_COMMAND
    lib::WORD value; // wartosc do wstawienia dla trybu INT_SINGLE_COMMAND

    long int current_absolute_position[MAX_SERVOS_NR];
    control_a_dof robot_control[MAX_SERVOS_NR];
    uint64_t hardware_error;

    status_of_a_dof robot_status[MAX_SERVOS_NR];
    // long int high_word;
};

typedef struct {
#ifdef __QNXNTO__
	struct sigevent event; // sygnalilzacja przerwania dla glownego watku
#endif
	common::motor_data md; // Dane przesylane z/do funkcji obslugi przerwania
} irq_data_t;

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
#define LM629_VIA_MICROCONTROLLER_MODE   0x4C01
#define MICROCONTROLLER_MODE             0x4C00
#define ZERO_ORDER                       0x0000
#define STOP_MOTORS                      0x0200 // Zatrzymanie silnikow (W.S. ???)

// tryby obslugi przerwania
#define INT_EMPTY 0 // obluga pusta
#define INT_SERVOING 1 // tryb regulacji osi
#define INT_SINGLE_COMMAND 2 // do synchronizacji, inicjacji, etc.
#define INT_CHECK_STATE 3 // do odczytu stanu z adresu 0x220

// ISA_CARD_OFFSET needs to be defined in hi_local.h
#define ADR_OF_SERVO_PTR          (0x305 + (ISA_CARD_OFFSET))
#define SERVO_COMMAND1_ADR        (0x200 + (ISA_CARD_OFFSET))
#define SERVO_COMMAND2_ADR        (0x202 + (ISA_CARD_OFFSET))
#define SERVO_REPLY_STATUS_ADR    (0x200 + (ISA_CARD_OFFSET))
#define SERVO_REPLY_INT_ADR       (0x202 + (ISA_CARD_OFFSET))
#define SERVO_REPLY_POS_LOW_ADR   (0x204 + (ISA_CARD_OFFSET))
#define SERVO_REPLY_POS_HIGH_ADR  (0x206 + (ISA_CARD_OFFSET))
#define SERVO_REPLY_REG_1_ADR     (0x208 + (ISA_CARD_OFFSET))
#define SERVO_REPLY_REG_2_ADR     (0x20A + (ISA_CARD_OFFSET))

class hardware_interface
{

protected:

    bool trace_resolver_zero;
    long int tick;// minimalny kwant czasu CPU
    control_a_dof robot_control[MAX_SERVOS_NR];
    status_of_a_dof robot_status[MAX_SERVOS_NR];

    int meassured_current[MAX_SERVOS_NR]; // by Y - zmierzona wartosc pradu
    long int current_absolute_position[MAX_SERVOS_NR];  // aktualne polozenia osi
    long int previous_absolute_position[MAX_SERVOS_NR]; // poprzednie polozenia osi
    double current_position_inc[MAX_SERVOS_NR];         // aktualny przyrost polozenia
    bool first; // true jesli pierwszy krok

public:
	manip_and_conv_effector &master;

    int irq_no;    // Numer przerwania sprzetowego
    int int_id;                 // Identyfikator obslugi przerwania
	//hardware_interface();    // Konstruktor
    hardware_interface( manip_and_conv_effector &_master );    // Konstruktor
    virtual ~hardware_interface( void );   // Destruktor
    virtual bool is_hardware_error ( void); // Sprawdzenie czy wystapil blad sprzetowy

    void insert_set_value ( int drive_number, double set_value);

    int get_current ( int drive_number );

    double get_increment ( int drive_number );

    long int get_position ( int drive_number ) ;


    virtual uint64_t read_write_hardware ( void );    // Obsluga sprzetu
    virtual void reset_counters ( void );  // Zerowanie licznikow polozenia

    virtual void start_synchro ( int drive_number ) ;

    virtual void finish_synchro ( int drive_number ) ;

    // oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
    int hi_int_wait (int inter_mode, int lag);

    bool is_impulse_zero ( int drive_number );

    void reset_position (int i);

}
; // koniec: class hardware_interface


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
