// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 postument
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6P_M_H
#define __HI_LOCAL_IRP6P_M_H

#include "edp/common/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_m {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

#define HI_RYDZ_INTR_TIMEOUT_HIGH 10000000 // by Y - timeout przerwania z szafy badz zegara

#define FIRST_SERVO_PTR           0xC1
#define INTERRUPT_GENERATOR_SERVO_PTR	 0xC0


#define ISA_CARD_OFFSET 0x20 // w zaleznosci od ustawienia na karcie isa

#define IRP6_POSTUMENT_AXIS_1_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_2_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_3_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_4_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_5_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_POSTUMENT_AXIS_6_MAX_CURRENT           0x24FF // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
// 13,7 j na amper

#define IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION  4000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION  2000.0  // Liczba impulsow enkodera na obrot walu - musi byc float

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

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


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
