// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota conveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_CONV_H
#define __HI_LOCAL_CONV_H

#include "edp/common/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

#define HI_RYDZ_INTR_TIMEOUT_HIGH 10000000 // by Y - timeout przerwania z szafy badz zegara

#define FIRST_SERVO_PTR           0xC0
#define INTERRUPT_GENERATOR_SERVO_PTR	 0xC0


#define ISA_CARD_OFFSET 0x20 // w zaleznosci od ustawienia na karcie isa

#define CONVEYOR_AXIS_1_MAX_CURRENT 0x2460
// 13,7 j na A

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


} // namespace conveyor
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
