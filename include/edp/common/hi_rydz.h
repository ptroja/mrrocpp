// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota iconveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_RYDZ_H
#define __HI_RYDZ_H

#include <stdint.h>
#include "edp/common/edp.h"

namespace mrrocpp {
namespace edp {
namespace common {

struct control_a_dof
{
    WORD adr_offset_plus_0;
    WORD adr_offset_plus_2;
};

struct status_of_a_dof
{
    WORD adr_offset_plus_0;
    WORD adr_offset_plus_2;
    WORD adr_offset_plus_4;
    WORD adr_offset_plus_6;
    WORD adr_offset_plus_8;
    WORD adr_offset_plus_a;
};


struct motor_data
{
    bool is_synchronised; // czy robot jest zsynchronizowany
    bool is_power_on; // czy wzmacniacze mocy sa wlaczone
    bool is_robot_blocked; // czy robot jest zablokowany

    int interrupt_mode;
    BYTE card_adress; // adres karty dla trybu INT_SINGLE_COMMAND
    WORD register_adress;  // adres rejestru dla trybu INT_SINGLE_COMMAND
    WORD value; // wartosc do wstawienia dla trybu INT_SINGLE_COMMAND

    long int current_absolute_position[MAX_SERVOS_NR];
    control_a_dof robot_control[MAX_SERVOS_NR];
    uint64_t hardware_error;

    status_of_a_dof robot_status[MAX_SERVOS_NR];
    // long int high_word;

};
// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

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
	edp_irp6s_and_conv_effector &master;

    int irq_no;    // Numer przerwania sprzetowego
    int int_id;                 // Identyfikator obslugi przerwania
	//hardware_interface();    // Konstruktor
    hardware_interface( edp_irp6s_and_conv_effector &_master );    // Konstruktor
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

#ifdef __cplusplus
extern "C"
{
#endif
    // pid_t far int_handler (void);  // Obsluga przerwania
    // by YOYEK & 7 - zastapic inna procedura obslugi prrzerwania

    const struct sigevent *
                int_handler (void *arg, int id); // by YOYEK & 7 - nowa forma z helpu

#ifdef __cplusplus
}
#endif

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
