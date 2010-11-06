// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------


#ifndef __REGULATOR_CONV_H
#define __REGULATOR_CONV_H

//#include "base/edp/servo_gr.h"
#include "base/edp/regulator.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

const double CONVEYOR35V_TO_CONVEYOR_VOLTAGE_RATIO = 0.60;

/*-----------------------------------------------------------------------*/
class NL_regulator_1_conv : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_1_conv(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_1
// ----------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
