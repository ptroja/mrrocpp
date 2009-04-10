// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP on_track
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_IRP6OT_H
#define __SG_IRP6OT_H

#include "edp/common/edp.h"
#include "edp/common/servo_gr.h"


namespace mrrocpp {
namespace edp {
namespace irp6ot {

#define IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS 10
#define IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE 100
// #define IRP6_ON_TRACK_GRIPPER_SUM_PWM_MIN 80

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
#define IRP6_ON_TRACK_AXE8_MAX_PWM_INCREMENT	1000



// Stale dla celow synchronizacji IRP6_ON_TRACK
// #define IRP6_ON_TRACK_SYNCHRO_STEP_COARSE -4*2*M_PI/IRP6_ON_TRACK_INC_PER_REVOLUTION
// #define IRP6_ON_TRACK_SYNCHRO_STEP_FINE   -1*2*M_PI/IRP6_ON_TRACK_INC_PER_REVOLUTION
#define IRP6_ON_TRACK_AXE_0_TO_5_SYNCHRO_STEP_COARSE -0.03
#define IRP6_ON_TRACK_AXE_0_TO_5_SYNCHRO_STEP_FINE   -0.007
#define IRP6_ON_TRACK_AXE_6_SYNCHRO_STEP_COARSE -0.05
#define IRP6_ON_TRACK_AXE_6_SYNCHRO_STEP_FINE   -0.05
#define IRP6_ON_TRACK_AXE_7_SYNCHRO_STEP_COARSE -0.5
#define IRP6_ON_TRACK_AXE_7_SYNCHRO_STEP_FINE   -0.2

#define POSTUMENT_TO_TRACK_VOLTAGE_RATIO 0.75


/*-----------------------------------------------------------------------*/
class NL_regulator_1_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_1_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_1
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_2_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_2_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_2
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_3_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_3_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_3
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_4_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_4_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_4
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_5_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    bool first;
    NL_regulator_5_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_5
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_6_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_6_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_6
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_7_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_7_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_7
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_8_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

    long gripper_blocked_start_time;
    int sum_of_currents, current_index;
    int currents [MAX_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS];


public:
    NL_regulator_8_irp6ot (BYTE reg_no, BYTE reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::irp6s_and_conv_effector &_master); // konstruktor

    virtual BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_8
// ----------------------------------------------------------------------








/************************ EDP_SPEAKER ****************************/
class servo_buffer  : public common::servo_buffer
{
    // Bufor polecen przysylanych z EDP_MASTER dla SERVO
    // Obiekt z algorytmem regulacji


    BYTE Move_a_step (void);         // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T

public:

    // output_buffer
    void get_all_positions (void);
    effector &master;

    servo_buffer (effector &_master);             // konstruktor
    ~servo_buffer (void);      // destruktor

    void synchronise (void);         // synchronizacja
    uint64_t compute_all_set_values (void);
    // obliczenie nastepnej wartosci zadanej dla wszystkich napedow


}
; // end: class servo_buffer
/************************ EDP_SPEAKER ****************************/

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp



#endif
