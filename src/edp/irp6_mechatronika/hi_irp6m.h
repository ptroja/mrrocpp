// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 mechatronika
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6M_H
#define __HI_LOCAL_IRP6M_H

#include <signal.h>
#include <time.h>

#include "edp/common/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

#define HI_RYDZ_INTR_TIMEOUT_HIGH 10000000 // by Y - timeout przerwania z szafy badz zegara

#define FIRST_SERVO_PTR           0xC1
#define INTERRUPT_GENERATOR_SERVO_PTR	 0xC1


#define ISA_CARD_OFFSET 0x10 // w zaleznosci od ustawienia na karcie isa

#define IRP6_MECHATRONIKA_AXIS_1_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_2_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_3_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_4_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_5_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_6_MAX_CURRENT           0x2410 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
// 13,7 j na amper

#define IRP6_MECHATRONIKA_AXIS_7_MAX_CURRENT           0x2420 // ustawienie pradu maksymalnego dla zacisku chwytaka
// 34,7 j na 100ma, streafa nieczulosci 40ma

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

#define SIG	SIGRTMIN

class hardware_interface: public common::hardware_interface {
public:
    hardware_interface (common::motor_driven_effector &_master, int _hi_irq_real,
    		unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high,
    		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr,
    		unsigned int _hi_isa_card_offset, const int _max_current[]);    // Konstruktor

}; // koniec: class hardware_interface

#ifdef __cplusplus
extern "C"
{
#endif
    const struct sigevent *
                int_handler (void *arg, int id);
#ifdef __cplusplus
}
#endif


} // namespace irp6m
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
