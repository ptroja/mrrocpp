// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6OT_TFG_H
#define __HI_LOCAL_IRP6OT_TFG_H

#include "robot/hi_rydz/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_tfg {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 4; // Dzielnik czestotliwosci przerwan

const int HI_RYDZ_INTR_TIMEOUT_HIGH = 10000000; // by Y - timeout przerwania z szafy badz zegara

const int FIRST_SERVO_PTR = 0xC7;
const int INTERRUPT_GENERATOR_SERVO_PTR = 0xC0;

const int ISA_CARD_OFFSET = 0x20; // w zaleznosci od ustawienia na karcie isa


// 13,7 j na amper

const int AXIS_8_MAX_CURRENT = 0x2427; // ustawienie pradu maksymalnego dla zacisku chwytaka
// by Y - UWAGA nieczulosc nieznana, rozdzielczosc do ustalenia
// 25,3 j na 100ma, strefa nieczulosci na poziomie 40ma




// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

class hardware_interface : public hi_rydz::HI_rydz
{

public:
			hardware_interface(common::motor_driven_effector &_master, int _hi_irq_real, unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high, unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, const int _max_current[]); // Konstruktor


}; // koniec: class hardware_interface

#ifdef __cplusplus
extern "C" {
#endif
const struct sigevent *
int_handler(void *arg, int id);
#ifdef __cplusplus
}
#endif

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
