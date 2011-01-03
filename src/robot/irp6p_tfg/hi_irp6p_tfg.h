// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 postument
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6P_TFG_H
#define __HI_LOCAL_IRP6P_TFG_H

#include <csignal>
#include <ctime>

#include "robot/hi_rydz/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_tfg {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

const long HI_RYDZ_INTR_TIMEOUT_HIGH = 10000000; // by Y - timeout przerwania z szafy badz zegara
const int FIRST_SERVO_PTR = 0xC7;
const int INTERRUPT_GENERATOR_SERVO_PTR = 0xC0;

const int ISA_CARD_OFFSET = 0x20; // w zaleznosci od ustawienia na karcie isa

const int AXIS_7_MAX_CURRENT = 0x2430; // ustawienie pradu maksymalnego dla zacisku chwytaka
// 34,7 j na 100ma, streafa nieczulosci 40ma


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

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
