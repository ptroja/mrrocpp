// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 13.01.2011
// -------------------------------------------------------------------------


#ifndef __REGULATOR_POLYCRANK_H
#define __REGULATOR_POLYCRANK_H

//#include "base/edp/servo_gr.h"
#include "base/edp/regulator.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

//const double POLYCRANK35V_TO_POLYCRANK_VOLTAGE_RATIO = 0.60;

/*-----------------------------------------------------------------------*/
class NL_regulator_polycrank : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_polycrank(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_1
// ----------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
