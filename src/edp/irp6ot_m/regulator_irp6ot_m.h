// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP on_track
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __REGULATOR_IRP6OT_M_H
#define __REGULATOR_IRP6OT_M_H

#include "edp/common/edp.h"
#include "edp/common/regulator.h"


namespace mrrocpp {
namespace edp {
namespace irp6ot_m {
class effector;


// os od ktorej startuje synchronizacja - numeracja od 0
#define IRP6OT_SYN_INIT_AXE 2

// ograniczenia przyrostu PWM dla IRP6_ON_TRACK
#define IRP6_ON_TRACK_AXE1_MAX_PWM_INCREMENT	1000
#define IRP6_ON_TRACK_AXE2_MAX_PWM_INCREMENT	1000
#define IRP6_ON_TRACK_AXE3_MAX_PWM_INCREMENT	1000
#define IRP6_ON_TRACK_AXE4_MAX_PWM_INCREMENT	1000
#define IRP6_ON_TRACK_AXE5_MAX_PWM_INCREMENT	1000
#define IRP6_ON_TRACK_AXE6_MAX_PWM_INCREMENT	1000
#define IRP6_ON_TRACK_AXE7_MAX_PWM_INCREMENT	1000




// Stale dla celow synchronizacji IRP6_ON_TRACK
// #define IRP6_ON_TRACK_SYNCHRO_STEP_COARSE -4*2*M_PI/IRP6_ON_TRACK_INC_PER_REVOLUTION
// #define IRP6_ON_TRACK_SYNCHRO_STEP_FINE   -1*2*M_PI/IRP6_ON_TRACK_INC_PER_REVOLUTION
#define IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_COARSE -0.03
#define IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_FINE   -0.007
#define IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_COARSE -0.05
#define IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_FINE   -0.05


#define POSTUMENT_TO_TRACK_VOLTAGE_RATIO 0.75


/*-----------------------------------------------------------------------*/
class NL_regulator_1_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_1_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_1
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_2_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_2_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_2
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_3_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_3_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_3
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_4_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_4_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_4
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_5_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    bool first;
    NL_regulator_5_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_5
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_6_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_6_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_6
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_7_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_7_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}; // end: class NL_regulator_7
// ----------------------------------------------------------------------


} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp



#endif
