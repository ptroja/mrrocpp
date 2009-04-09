// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------


#ifndef __SG_IRP6P_AND_CONV_H
#define __SG_IRP6P_AND_CONV_H

#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace common {

// ograniczenia przyrostu PWM dla CONVEYOR
#define CONVEYOR_AXE1_MAX_PWM_INCREMENT	1000


#define IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS 10
#define IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE 250
// #define IRP6_POSTUMENT_GRIPPER_SUM_PWM_MIN 80


// ograniczenia przyrostu PWM dla IRP6_POSTUMENT
#define IRP6_POSTUMENT_AXE1_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE2_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE3_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE4_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE5_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE6_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE7_MAX_PWM_INCREMENT	1000
#define IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT	1000


// Stale dla celow synchronizacji IRP6_POSTUMENT
// #define IRP6_POSTUMENT_SYNCHRO_STEP_COARSE -4*2*M_PI/IRP6_POSTUMENT_INC_PER_REVOLUTION
// #define IRP6_POSTUMENT_SYNCHRO_STEP_FINE   -1*2*M_PI/IRP6_POSTUMENT_INC_PER_REVOLUTION
#define IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_COARSE -0.03
#define IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_FINE   -0.007
#define IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_COARSE -0.05
#define IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_FINE   -0.05
#define IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_COARSE -0.5
#define IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_FINE   -0.2


/*-----------------------------------------------------------------------*/
class NL_regulator_1_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_1_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_1
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_2_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_2_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_2
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_3_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_3_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_3
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_4_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_4_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_4
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_5_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    bool first;
    NL_regulator_5_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_5
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_6_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_6_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_6
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_7_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_7_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_7
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_8_irp6p: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

    long gripper_blocked_start_time;
    int sum_of_currents, current_index;
    int currents [MAX_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS];


public:
    NL_regulator_8_irp6p (BYTE reg_no, BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::edp_irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_8
// ----------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
