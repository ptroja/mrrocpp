// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------


#ifndef __REGULATOR_CONV_H
#define __REGULATOR_CONV_H

//#include "edp/common/servo_gr.h"
#include "edp/common/regulator.h"




namespace mrrocpp {
namespace edp {
namespace common {

// ograniczenia przyrostu PWM dla CONVEYOR
#define CONVEYOR_AXE1_MAX_PWM_INCREMENT	1000






/*-----------------------------------------------------------------------*/
class NL_regulator_1_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_1_irp6p (uint8_t reg_no, uint8_t reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_1
// ----------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
