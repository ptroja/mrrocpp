// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------


#ifndef __REGULATOR_IRP6P_M_H
#define __REGULATOR_IRP6P_M_H

//#include "base/edp/servo_gr.h"
#include "base/edp/regulator.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_m {

// Stale dla celow synchronizacji IRP6_POSTUMENT
// #define IRP6_POSTUMENT_SYNCHRO_STEP_COARSE -4*2*M_PI/IRP6_POSTUMENT_INC_PER_REVOLUTION
// #define IRP6_POSTUMENT_SYNCHRO_STEP_FINE   -1*2*M_PI/IRP6_POSTUMENT_INC_PER_REVOLUTION
const double AXIS_0_TO_5_SYNCHRO_STEP_COARSE = -0.03;
const double AXIS_0_TO_5_SYNCHRO_STEP_FINE = -0.007;
const double AXIS_6_SYNCHRO_STEP_COARSE = -0.05;
const double AXIS_6_SYNCHRO_STEP_FINE = -0.05;

const double POSTUMENT35V_TO_POSTUMENT_VOLTAGE_RATIO = 0.60;	// Preskaler dla osi 1-3
const double POSTUMENT35V_TO_POSTUMENT_VOLTAGE_RATIO_2 = 0.40;	// Preskaler dla osi 5-7

/*-----------------------------------------------------------------------*/
class NL_regulator_2_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_2_irp6p(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_2
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_3_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_3_irp6p(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_3
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_4_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_4_irp6p(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_4
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_5_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
	bool first;
			NL_regulator_5_irp6p(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_5
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_6_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_6_irp6p(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_6
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_7_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji

public:
			NL_regulator_7_irp6p(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_7
// ----------------------------------------------------------------------


} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
